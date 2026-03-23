# Topic 名称
RAW_ACTION_TOPIC = '/twin/raw_action'
HW_CONTROL_TOPIC = '/twin/hw/control'
HW_STATE_TOPIC = '/twin/hw/state'
SIM_STATE_TOPIC = '/twin/sim/state'

NUM_JOINTS = 12
ACTION_SLIDER_MIN = -100
ACTION_SLIDER_MAX = 100
ACTION_VALUE_SCALE = 100.0

JOINT_NAMES = [
    'LF_A', 'LR_A', 'RF_A', 'RR_A',
    'LF_F', 'LR_F', 'RF_F', 'RR_F',
    'LF_K', 'LR_K', 'RF_K', 'RR_K',
]