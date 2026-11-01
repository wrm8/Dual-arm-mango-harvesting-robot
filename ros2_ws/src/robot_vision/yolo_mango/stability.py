"""Check consistency against the first frame in the current observation window."""
import math


class StabilityGate:
    def __init__(self, cfg):
        self.cfg = cfg
        if cfg['stable_frames'] < 1:
            raise ValueError('stable_frames must be >= 1')
        self.reset()

    def reset(self):
        self.anchor = None
        self.count = 0

    def update(self, result, fruits):
        if result['status'] != 'confirmed':
            self.reset()
            return False
        by_id = {f.id: f for f in fruits}
        targets = {t['id']: tuple(t['reasons']) for t in result['targets']}
        state = {}
        for slot, fruit_id in result['slots'].items():
            f = by_id[fruit_id]
            state[slot] = (f.color, targets.get(f.id, ()), f.cx, f.cy, f.depth_m)
        consistent = self.anchor is not None and state.keys() == self.anchor.keys()
        if consistent:
            for slot, current in state.items():
                previous = self.anchor[slot]
                if current[:2] != previous[:2]:
                    consistent = False
                    break
                if math.hypot(current[2] - previous[2], current[3] - previous[3]) > self.cfg['stable_center_px']:
                    consistent = False
                    break
                a, b = current[4], previous[4]
                if (a is None) != (b is None) or (a is not None and abs(a-b) > self.cfg['stable_depth_m']):
                    consistent = False
                    break
        if not consistent:
            self.anchor = state
            self.count = 1
        else:
            self.count += 1
        return self.count >= self.cfg['stable_frames']
