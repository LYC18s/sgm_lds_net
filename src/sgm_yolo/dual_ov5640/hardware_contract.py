"""Validate display layout and reset wiring before downloading/accessing PL."""


def validate_hdmi_layout(root, panel_y=None):
    """Accept known capture layouts, including the centered LDS-Net display.

    Panel placement changes where HDMI consumes a frame, not its DDR capture
    stride or dimensions. Keep the horizontal capture ABI strict and require
    both displayed panels to use the same supported vertical origin.
    """
    modules = {m.get('INSTANCE'): m for m in root.findall('.//MODULE')}
    right = ('axi_vdma_0' in modules, 'axi_vdma_1' in modules)
    if right[0] != right[1]:
        raise ValueError('Incomplete right RGB DMA pair in HWH')
    compositor = modules.get('triple_rgb_compositor_0')
    values = {} if compositor is None else {
        p.get('NAME'): p.get('VALUE') for p in compositor.findall('./PARAMETERS/PARAMETER')}
    keys = ('LEFT_X', 'LEFT_Y', 'DISP_X', 'DISP_Y')
    try:
        actual = tuple(int(values[k], 0) for k in keys)
        right_enable = int(values['RIGHT_ENABLE'], 0)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('HDMI layout metadata is missing or invalid in HWH') from exc
    if not right[0] and right_enable != 0:
        raise ValueError('Compact HDMI must disable the right RGB panel')
    supported = ((0, 0, 1280, 0),) if right[0] else ((0, 0, 640, 0), (0, 300, 640, 300))
    if actual not in supported:
        raise ValueError('HDMI layout does not match the RGB capture ABI: '
                         'actual %s; supported %s' % (actual, supported))
    if panel_y is not None and (actual[1] != int(panel_y) or actual[3] != int(panel_y)):
        raise ValueError('HDMI panel Y does not match the board manifest: '
                         'HWH=%s, manifest=%s' % ((actual[1], actual[3]), panel_y))
    return dict(zip(keys, actual))


def validate_lds_net_reset(root):
    modules = {m.get('INSTANCE'): m for m in root.findall('.//MODULE')}
    reset = modules.get('lds_net_reset')
    if reset is None:
        raise RuntimeError('Missing LDS-Net reset metadata; install the complete matching board package')
    params = {p.get('NAME'): p.get('VALUE') for p in reset.findall('./PARAMETERS/PARAMETER')}
    ports = {p.get('NAME'): p.get('SIGNAME') for p in reset.findall('./PORTS/PORT')}
    constants = {}
    for m in modules.values():
        if m.get('MODTYPE') == 'xlconstant':
            values = {p.get('NAME'): p.get('VALUE') for p in m.findall('./PARAMETERS/PARAMETER')}
            if int(values.get('CONST_WIDTH', '0'), 0) == 1:
                for p in m.findall('./PORTS/PORT'):
                    if p.get('DIR') == 'O' and p.get('SIGNAME'):
                        constants[p.get('SIGNAME')] = int(values['CONST_VAL'], 0)
    active = int(params.get('C_AUX_RESET_HIGH', '-1'), 0)
    auxiliary = constants.get(ports.get('aux_reset_in'))
    debug = constants.get(ports.get('mb_debug_sys_rst'))
    if active not in (0, 1) or auxiliary != 1-active or debug != 0:
        raise RuntimeError('Unsafe LDS-Net reset wiring: auxiliary reset is asserted or unverified. '
                           'Do not access its MMIO; install the complete reset-v4 or later bit/HWH package')
    return dict(aux_active_high=active, aux_constant=auxiliary, debug_constant=debug)
