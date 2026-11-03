"""Explicit live-board AXI DMA Simple adapter. Import alone performs no I/O.

Use a dedicated, exclusively owned PG021 MMIO object, not an active PYNQ DMA driver.
No allocation, Overlay download, reset, buffer free or filesystem write is automatic.
Caller-provided MMIO must have bounded read/write; Python cannot time out a hung bus.
"""
import time

from axis_core import REGISTERS, integer, positive

# Strong references survive exceptions and prevent buffer finalizers from freeing live CMA.
QUARANTINE = []


class TransferFailure(RuntimeError):
    def __init__(self, message, evidence):
        super().__init__(message)
        self.evidence = evidence


def validate_parameters(parameters, directions):
    p = {key.lower(): integer(value) for key, value in parameters.items()
         if key.lower() in ('c_include_sg', 'c_micro_dma', 'c_sg_length_width', 'c_addr_width',
                            'c_include_mm2s', 'c_include_s2mm', 'c_include_mm2s_dre',
                            'c_include_s2mm_dre', 'c_m_axi_mm2s_data_width', 'c_m_axi_s2mm_data_width')}
    for name in ('c_include_sg', 'c_micro_dma', 'c_sg_length_width', 'c_addr_width'):
        if name not in p:
            raise ValueError('Missing exported hardware parameter %s; do not guess' % name)
    if p['c_include_sg'] or p['c_micro_dma']:
        raise ValueError('Only non-Micro AXI DMA Simple is supported')
    if not 8 <= p['c_sg_length_width'] <= 26 or not 32 <= p['c_addr_width'] <= 64:
        raise ValueError('Unsupported length/address width')
    for direction in directions:
        for key in ('c_include_' + direction, 'c_include_' + direction + '_dre',
                    'c_m_axi_' + direction + '_data_width'):
            if key not in p:
                raise ValueError('Missing exported hardware parameter %s' % key)
        if p['c_include_' + direction] != 1:
            raise ValueError('Requested DMA channel is not enabled')
        width = p['c_m_axi_' + direction + '_data_width']
        if width not in (32, 64, 128, 256, 512, 1024):
            raise ValueError('Unsupported memory-map width')
    return p


def snapshot(mmio, directions=('mm2s', 's2mm')):
    """Read known PG021 registers only; caller must verify the IP identity first."""
    names = {direction.upper() for direction in directions}
    return dict(ip_type='axi_dma', registers={name: '0x%08x' % mmio.read(offset)
                for name, offset in REGISTERS.items() if name.split('_')[0] in names},
                monotonic_s=time.monotonic())


def sync(buffer, operation):
    if getattr(buffer, 'coherent', None) is True:
        return
    method = getattr(buffer, operation, None)
    if not callable(method):
        raise ValueError('Non-coherent buffer needs %s()' % operation)
    method()


def run_simple(mmio, parameters, *, tx=None, rx=None, tx_bytes=None, rx_bytes=None,
               expected_rx_bytes=None, timeout_s=2.0, poll_s=.001, start_ip=None,
               exclusive_owner=False, ip_type=None):
    """Bounded polling after RX-first launch; the only supported live mutation entry.

    parameters must come from matching HWH/ip_dict, not the example file. start_ip,
    if supplied, must be nonblocking and configure a previously reviewed IP contract.
    timeout covers launch/polling, not a blocked MMIO transaction or callback.
    Any exception after hardware writes quarantines all buffers without auto-reset.
    """
    if not exclusive_owner or ip_type != 'axi_dma':
        raise ValueError('Explicit exclusive ownership and axi_dma identity are required')
    if QUARANTINE:
        raise RuntimeError('A prior transfer is quarantined; complete documented recovery first')
    positive(timeout_s, 'timeout_s')
    positive(poll_s, 'poll_s')
    channels = [(name, buffer, length) for name, buffer, length in
                (('s2mm', rx, rx_bytes), ('mm2s', tx, tx_bytes)) if buffer is not None]
    if not channels:
        raise ValueError('At least one channel is required')
    directions = tuple(item[0] for item in channels)
    p = validate_parameters(parameters, directions)
    prepared = []
    for direction, buffer, requested in channels:
        nbytes = int(buffer.nbytes) if requested is None else integer(requested)
        if not 0 < nbytes <= min(int(buffer.nbytes), (1 << p['c_sg_length_width']) - 1):
            raise ValueError('Transfer length exceeds buffer or hardware maximum')
        address = getattr(buffer, 'device_address', getattr(buffer, 'physical_address', None))
        if address is None:
            raise ValueError('Use a device-addressable buffer allocated by PYNQ')
        address = int(address)
        if address < 0 or address + nbytes > 1 << p['c_addr_width']:
            raise ValueError('Buffer address exceeds hardware address width')
        alignment = p['c_m_axi_' + direction + '_data_width'] // 8
        if not p['c_include_' + direction + '_dre'] and address % alignment:
            raise ValueError('Unaligned address without DRE')
        if hasattr(buffer, 'flags') and not buffer.flags['C_CONTIGUOUS']:
            raise ValueError('Non-contiguous buffer view')
        # Validate both operations before any hardware write.
        for operation in ('flush', 'invalidate') if direction == 's2mm' else ('flush',):
            if getattr(buffer, 'coherent', None) is not True and not callable(getattr(buffer, operation, None)):
                raise ValueError('Missing buffer cache operation %s' % operation)
        prepared.append((direction, buffer, nbytes, address))
    if tx is not None and rx is not None:
        a, b = prepared
        if max(a[3], b[3]) < min(a[3] + a[2], b[3] + b[2]):
            raise ValueError('TX/RX DMA buffer ranges overlap')
    if expected_rx_bytes is not None:
        if rx is None or type(expected_rx_bytes) is not int or not 0 < expected_rx_bytes <= prepared[0][2]:
            raise ValueError('Invalid expected_rx_bytes')
    before = snapshot(mmio, directions)
    for direction in directions:
        status = integer(before['registers'][direction.upper() + '_DMASR'])
        if status & 0x4778:
            raise ValueError('DMA has errors or SG enabled; preserve evidence and recover separately')
        if not status & 3:
            raise ValueError('DMA is neither halted nor idle; refuse to take over an active channel')
        control = mmio.read(0 if direction == 'mm2s' else 0x30)
        if control & 0x14:
            raise ValueError('DMA reset/cyclic mode is active')
    for _, buffer, _, _ in prepared:
        # RX flush also prevents dirty CPU lines later overwriting DMA-written data.
        sync(buffer, 'flush')
    started = time.monotonic()
    deadline = started + timeout_s
    mutated = False
    history = []
    try:
        for direction, buffer, nbytes, address in prepared:
            base = 0 if direction == 'mm2s' else 0x30
            mutated = True
            mmio.write(base + 4, 0x1000)  # Clear old completion only; preserve error evidence.
            mmio.write(base, (mmio.read(base) & ~0x7000) | 1)  # Polling, no IRQ owner.
            while mmio.read(base + 4) & 1:
                if time.monotonic() >= deadline:
                    raise TimeoutError('Channel failed to leave Halted')
                time.sleep(poll_s)
            mmio.write(base + 0x18, address & 0xffffffff)
            if p['c_addr_width'] > 32:
                mmio.write(base + 0x1c, address >> 32)
            mmio.write(base + 0x28, nbytes)  # RX is armed before TX length is written.
        if start_ip is not None:
            start_ip()
        while True:
            statuses = {direction: mmio.read(4 if direction == 'mm2s' else 0x34) for direction in directions}
            if not history or history[-1]['status'] != statuses:
                history.append(dict(elapsed_s=time.monotonic()-started, status=statuses))
            if any(value & 0x4770 for value in statuses.values()):
                raise RuntimeError('DMA hardware error; inspect saved status')
            if all(value & 2 and value & 0x1000 and not value & 1 for value in statuses.values()):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError('DMA completion deadline exceeded; TLAST is only one possible cause')
            time.sleep(poll_s)
        actual = mmio.read(0x58) if rx is not None else None
        if rx is not None:
            sync(rx, 'invalidate')
        result = dict(kind='board_transfer', status='pass', scope='python_e2e',
                      seconds=time.monotonic()-started, tx_bytes=tx_bytes if tx_bytes is not None else int(tx.nbytes) if tx is not None else 0,
                      rx_actual_bytes=actual, before=before, after=snapshot(mmio, directions), history=history)
        if expected_rx_bytes is not None and actual != expected_rx_bytes:
            result['status'] = 'fail'
            result['reason'] = 'RX byte count differs from interface contract'
        return result
    except BaseException as exc:
        if mutated:
            try:
                after = snapshot(mmio, directions)
            except Exception as read_error:
                after = {'unavailable': str(read_error)}
            evidence = dict(kind='board_transfer', status='fail', reason=str(exc), before=before,
                            after=after, history=history, buffers_retained=True)
            QUARANTINE.append(dict(mmio=mmio, buffers=[item[1] for item in prepared], evidence=evidence))
            if isinstance(exc, (KeyboardInterrupt, SystemExit)):
                raise
            raise TransferFailure(str(exc), evidence) from exc
        raise


def release_quarantine_after_recovery(index, *, upstream_quiescent=False):
    """Only after caller has stopped upstream and drained/reset owners per PG021.

    This function never performs recovery writes or frees buffers. Keep the returned
    buffers alive until all non-DMA owners are also known to be quiescent.
    """
    if not upstream_quiescent:
        raise ValueError('Upstream and all buffer owners must first be confirmed quiescent')
    entry = QUARANTINE[index]
    directions = tuple(name.split('_')[0].lower() for name in entry['evidence']['before']['registers'] if name.endswith('_DMASR'))
    for direction in directions:
        base = 0 if direction == 'mm2s' else 0x30
        if mmio_read(entry['mmio'], base) & 1 or not mmio_read(entry['mmio'], base + 4) & 1:
            raise RuntimeError('Require RS=0 and Halted=1 before releasing quarantine')
    return QUARANTINE.pop(index)


def mmio_read(mmio, offset):
    return int(mmio.read(offset))
