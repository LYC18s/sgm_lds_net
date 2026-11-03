# The board build runs on a 16-core / 24-thread machine. Eight workers avoid
# serial placement/routing while leaving memory for the desktop and simulation.
set_param general.maxThreads 8
