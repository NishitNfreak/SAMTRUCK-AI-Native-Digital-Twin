# SAMTRUCK Research Decisions

## Deferred AI and Prediction

Phase 1 contains no AI, prediction, adaptive control, Kalman filter, or stale-data extrapolation. Later work should begin with a Random Forest prediction baseline, consider LSTM only as an advanced trajectory model, and consider DQN only for later update-policy decisions balancing accuracy, communication, and compute cost.

Future training labels from SUMO ground truth belong only to offline data collection. They must never become live Digital Twin inputs.

## Experimental Controls

- Compare baseline CACC and any future adaptive controller under identical scenario seeds and conditions.
- Keep training, validation, and test scenarios separate.
- Evaluate mixed traffic, V2X reliability, redundant sensing, and later longitudinal/lateral integration.
- Report prediction error, safety, string stability, communication, and computation metrics. Do not report literature findings as SAMTRUCK results.

## Technical Constraints for Later Phases

- Safe gap must use conservative relative stopping-distance assumptions, not communication delay alone.
- Calculate jerk using real simulation `dt`.
- Detect collisions from SUMO collision events, not negative gap alone.
- Measure string stability as leader-to-follower disturbance amplification.
- Do not claim fuel or energy results without a verified configured SUMO model.
