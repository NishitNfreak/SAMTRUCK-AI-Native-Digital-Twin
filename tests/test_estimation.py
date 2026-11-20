"""Deterministic Phase 2.1A tests for passive estimator primitives."""

from __future__ import annotations

from pathlib import Path
import unittest

import numpy as np

from samtruck.estimation import (
    EstimatorConfig,
    RadarTargetTracker,
    RadarTrackerConfig,
    VehicleStateEstimator,
    propagate_predecessor_v2x,
)
from samtruck.v2x import PendingV2XChannel, V2XMessage


class VehicleStateEstimatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.config = EstimatorConfig(
            jerk_process_variance=1.0,
            position_measurement_variance=0.01,
            speed_measurement_variance=0.01,
            acceleration_measurement_variance=0.01,
            initial_position_variance=100.0,
            initial_speed_variance=1.0,
            initial_acceleration_variance=1.0,
            long_gap_s=0.5,
        )

    def test_constant_speed_prediction(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        estimator.update(0.0, independent_position_m=10.0, speed_mps=20.0, acceleration_mps2=0.0)
        state = estimator.predict_only(2.0)
        self.assertAlmostEqual(state.estimated_position_m, 50.0, places=6)
        self.assertAlmostEqual(state.estimated_speed_mps, 20.0, places=6)

    def test_constant_acceleration_prediction(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        estimator.update(0.0, independent_position_m=0.0, speed_mps=10.0, acceleration_mps2=2.0)
        state = estimator.predict_only(3.0)
        self.assertAlmostEqual(state.estimated_position_m, 39.0, places=6)
        self.assertAlmostEqual(state.estimated_speed_mps, 16.0, places=6)

    def test_variable_timestep(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        estimator.update(0.0, speed_mps=10.0, acceleration_mps2=1.0)
        estimator.predict_only(0.25)
        state = estimator.predict_only(1.0)
        self.assertAlmostEqual(state.estimated_speed_mps, 11.0, places=6)

    def test_speed_acceleration_and_independent_position_rows(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        estimator.update(0.0, speed_mps=1.0)
        speed_only = estimator.update(1.0, speed_mps=5.0)
        self.assertEqual(speed_only.measurement_update_mask, ("speed",))
        acceleration_only = estimator.update(2.0, acceleration_mps2=2.0)
        self.assertEqual(acceleration_only.measurement_update_mask, ("acceleration",))
        position = estimator.update(3.0, independent_position_m=100.0, position_variance=0.0001)
        self.assertEqual(position.measurement_update_mask, ("position",))
        self.assertAlmostEqual(position.estimated_position_m, 100.0, places=2)

    def test_missing_measurement_is_not_zero_and_covariance_grows(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        initial = estimator.update(0.0, speed_mps=10.0)
        predicted = estimator.predict_only(1.0)
        self.assertEqual(predicted.status, "degraded_long_gap_predicted")
        self.assertGreater(predicted.speed_std_mps, initial.speed_std_mps)
        self.assertEqual(predicted.measurement_update_mask, ())

    def test_invalid_inputs_and_out_of_order_timestamp(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        with self.assertRaises(ValueError):
            estimator.update(0.0, speed_mps=float("nan"))
        estimator.update(1.0, speed_mps=10.0)
        with self.assertRaises(ValueError):
            estimator.update(1.0, speed_mps=10.0, speed_variance=-1.0)
        old = estimator.update(0.5, speed_mps=20.0)
        self.assertEqual(old.status, "invalid_timestamp")
        self.assertAlmostEqual(old.timestamp_s, 1.0)
        same_time = estimator.update(1.0, speed_mps=11.0)
        self.assertEqual(same_time.status, "updated_same_timestamp")

    def test_initialization_with_missing_speed_or_acceleration(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        uninitialized = estimator.update(0.0, independent_position_m=5.0)
        self.assertFalse(uninitialized.initialized)
        initialized = estimator.update(1.0, speed_mps=6.0)
        self.assertTrue(initialized.initialized)
        self.assertEqual(initialized.status, "partially_initialized")
        self.assertGreater(initialized.acceleration_std_mps2, initialized.speed_std_mps)

    def test_component_freshness_full_and_partial_initialization(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        full = estimator.update(1.0, speed_mps=6.0, acceleration_mps2=0.5)
        self.assertEqual(full.last_speed_measurement_timestamp_s, 1.0)
        self.assertEqual(full.speed_measurement_age_s, 0.0)
        self.assertTrue(full.speed_measurement_ever_received)
        self.assertEqual(full.speed_estimate_provenance, "measurement_updated")
        self.assertEqual(full.last_acceleration_measurement_timestamp_s, 1.0)
        self.assertEqual(full.acceleration_measurement_age_s, 0.0)
        self.assertEqual(full.acceleration_estimate_provenance, "measurement_updated")
        self.assertIsNone(full.independent_position_measurement_age_s)
        self.assertEqual(full.position_estimate_provenance, "not_meaningfully_initialized")

        speed_only = VehicleStateEstimator("truck_1", self.config).update(2.0, speed_mps=7.0)
        self.assertEqual(speed_only.speed_measurement_age_s, 0.0)
        self.assertIsNone(speed_only.last_acceleration_measurement_timestamp_s)
        self.assertIsNone(speed_only.acceleration_measurement_age_s)
        self.assertFalse(speed_only.acceleration_measurement_ever_received)
        self.assertEqual(speed_only.acceleration_estimate_provenance, "not_meaningfully_initialized")

    def test_component_ages_are_independent_and_accumulate_through_predictions(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        estimator.update(0.0, speed_mps=10.0, acceleration_mps2=1.0)
        speed_only = estimator.update(1.0, speed_mps=11.0)
        self.assertEqual(speed_only.speed_measurement_age_s, 0.0)
        self.assertEqual(speed_only.acceleration_measurement_age_s, 1.0)
        self.assertEqual(speed_only.acceleration_estimate_provenance, "prediction_only")
        acceleration_only = estimator.update(2.0, acceleration_mps2=2.0)
        self.assertEqual(acceleration_only.speed_measurement_age_s, 1.0)
        self.assertEqual(acceleration_only.acceleration_measurement_age_s, 0.0)
        self.assertEqual(acceleration_only.speed_estimate_provenance, "prediction_only")
        predicted = estimator.predict_only(3.5)
        self.assertEqual(predicted.speed_measurement_age_s, 2.5)
        self.assertEqual(predicted.acceleration_measurement_age_s, 1.5)
        self.assertEqual(predicted.speed_estimate_provenance, "prediction_only")
        self.assertEqual(predicted.acceleration_estimate_provenance, "prediction_only")

    def test_independent_position_only_refreshes_when_explicitly_supplied(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        initial = estimator.update(0.0, speed_mps=8.0, acceleration_mps2=0.0)
        self.assertIsNone(initial.independent_position_measurement_age_s)
        position = estimator.update(1.0, independent_position_m=20.0)
        self.assertEqual(position.last_independent_position_measurement_timestamp_s, 1.0)
        self.assertEqual(position.independent_position_measurement_age_s, 0.0)
        self.assertTrue(position.independent_position_measurement_ever_received)
        self.assertEqual(position.position_estimate_provenance, "measurement_updated")
        predicted = estimator.predict_only(2.0)
        self.assertEqual(predicted.independent_position_measurement_age_s, 1.0)
        self.assertEqual(predicted.position_estimate_provenance, "prediction_only")

    def test_same_and_invalid_timestamp_do_not_falsely_refresh_missing_components(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        estimator.update(1.0, speed_mps=10.0, acceleration_mps2=1.0)
        same_time = estimator.update(1.0, speed_mps=11.0)
        self.assertEqual(same_time.speed_measurement_age_s, 0.0)
        self.assertEqual(same_time.acceleration_measurement_age_s, 0.0)
        self.assertEqual(same_time.acceleration_estimate_provenance, "prediction_only")
        old = estimator.update(0.5, speed_mps=99.0, acceleration_mps2=99.0)
        self.assertEqual(old.status, "invalid_timestamp")
        self.assertEqual(old.timestamp_s, 1.0)
        self.assertEqual(old.last_speed_measurement_timestamp_s, 1.0)
        self.assertEqual(old.last_acceleration_measurement_timestamp_s, 1.0)
        self.assertEqual(old.speed_estimate_provenance, "invalid_timestamp_no_update")
        self.assertEqual(old.acceleration_estimate_provenance, "invalid_timestamp_no_update")

    def test_uninitialized_components_are_unavailable(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        uninitialized = estimator.update(0.0, independent_position_m=5.0)
        self.assertIsNone(uninitialized.speed_measurement_age_s)
        self.assertIsNone(uninitialized.acceleration_measurement_age_s)
        self.assertEqual(uninitialized.speed_estimate_provenance, "unavailable")
        self.assertEqual(uninitialized.acceleration_estimate_provenance, "unavailable")

    def test_covariance_is_symmetric_positive_semidefinite(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        estimator.update(0.0, speed_mps=10.0, acceleration_mps2=0.0)
        state = estimator.update(0.7, speed_mps=11.0, acceleration_mps2=0.5)
        covariance = np.asarray(state.covariance)
        np.testing.assert_allclose(covariance, covariance.T, atol=1e-10)
        self.assertGreaterEqual(float(np.linalg.eigvalsh(covariance).min()), -1e-9)

    def test_integrated_white_jerk_process_covariance_exact_values_and_scaling(self) -> None:
        estimator = VehicleStateEstimator("truck_0", EstimatorConfig(jerk_process_variance=2.0))
        for dt_s in (0.5, 2.0):
            expected = 2.0 * np.array([
                [dt_s**5 / 20.0, dt_s**4 / 8.0, dt_s**3 / 6.0],
                [dt_s**4 / 8.0, dt_s**3 / 3.0, dt_s**2 / 2.0],
                [dt_s**3 / 6.0, dt_s**2 / 2.0, dt_s],
            ])
            actual = estimator.process_covariance(dt_s)
            np.testing.assert_allclose(actual, expected, atol=1e-12)
            np.testing.assert_allclose(actual, actual.T, atol=1e-12)
            self.assertGreaterEqual(float(np.linalg.eigvalsh(actual).min()), -1e-12)
        q_half = estimator.process_covariance(0.5)
        q_two = estimator.process_covariance(2.0)
        self.assertAlmostEqual(q_two[2, 2] / q_half[2, 2], 4.0)  # dt scaling, not dt².
        self.assertAlmostEqual(q_two[0, 0] / q_half[0, 0], 4.0**5)

    def test_prediction_only_covariance_grows_with_correct_process_noise(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        initial = estimator.update(0.0, speed_mps=10.0, acceleration_mps2=0.0)
        predicted = estimator.predict_only(2.0)
        self.assertGreater(predicted.acceleration_std_mps2, initial.acceleration_std_mps2)
        self.assertGreater(predicted.speed_std_mps, initial.speed_std_mps)

    def test_measurement_recovery_reduces_uncertainty_after_prediction_gap(self) -> None:
        estimator = VehicleStateEstimator("truck_0", self.config)
        estimator.update(0.0, speed_mps=10.0, acceleration_mps2=0.0)
        predicted = estimator.predict_only(2.0)
        recovered = estimator.update(2.1, speed_mps=10.0, acceleration_mps2=0.0)
        self.assertEqual(predicted.status, "degraded_long_gap_predicted")
        self.assertEqual(recovered.measurement_update_mask, ("speed", "acceleration"))
        self.assertLess(recovered.speed_std_mps, predicted.speed_std_mps)
        self.assertLess(recovered.acceleration_std_mps2, predicted.acceleration_std_mps2)


class RadarTargetTrackerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tracker = RadarTargetTracker(RadarTrackerConfig(
            relative_acceleration_process_variance=1.0,
            range_measurement_variance=0.01,
            relative_speed_measurement_variance=0.01,
            long_gap_s=0.5,
        ))

    def test_positive_closing_speed_reduces_range(self) -> None:
        self.tracker.update("target_a", 0.0, range_m=100.0, relative_speed_mps=5.0)
        state = self.tracker.predict_missing("target_a", 2.0)
        self.assertAlmostEqual(state.estimated_range_m, 90.0, places=6)
        self.assertEqual(state.status, "degraded_long_gap_missing")

    def test_range_only_relative_speed_only_and_dropout(self) -> None:
        range_only = self.tracker.update("target_a", 0.0, range_m=30.0)
        self.assertEqual(range_only.measurement_update_mask, ("range",))
        speed_only = self.tracker.update("target_a", 0.1, relative_speed_mps=2.0)
        self.assertEqual(speed_only.measurement_update_mask, ("relative_speed",))
        dropout = self.tracker.predict_missing("target_a", 0.2)
        self.assertEqual(dropout.status, "predicted_missing")
        self.assertFalse(dropout.radar_valid)

    def test_partial_initialization_provenance_is_explicit(self) -> None:
        range_only = self.tracker.update("range_target", 0.0, range_m=30.0)
        self.assertEqual(range_only.status, "initialized_range_only_prior_relative_speed")
        self.assertEqual(range_only.observed_components, ("range",))
        self.assertEqual(range_only.unobserved_prior_components, ("relative_speed",))
        self.assertEqual(range_only.last_measurement_timestamp_s, 0.0)
        self.assertTrue(range_only.last_update_was_measurement)

        speed_only = self.tracker.update("speed_target", 0.0, relative_speed_mps=2.0)
        self.assertEqual(speed_only.status, "initialized_relative_speed_only_prior_range")
        self.assertEqual(speed_only.observed_components, ("relative_speed",))
        self.assertEqual(speed_only.unobserved_prior_components, ("range",))

    def test_old_track_is_not_reported_as_fresh_measurement(self) -> None:
        self.tracker.update("target_a", 0.0, range_m=30.0, relative_speed_mps=2.0)
        predicted = self.tracker.predict_missing("target_a", 1.0)
        self.assertFalse(predicted.radar_valid)
        self.assertFalse(predicted.last_update_was_measurement)
        self.assertAlmostEqual(predicted.last_measurement_timestamp_s, 0.0)
        self.assertAlmostEqual(predicted.prediction_timestamp_s, 1.0)
        self.assertAlmostEqual(predicted.track_age_s, 1.0)
        queried = self.tracker.state("target_a")
        self.assertEqual(queried.status, "prediction_only")
        self.assertFalse(queried.radar_valid)
        self.assertFalse(queried.last_update_was_measurement)

    def test_separate_target_ids_and_invalid_timestamp(self) -> None:
        self.tracker.update("cut_in", 1.0, range_m=10.0, relative_speed_mps=1.0)
        self.tracker.update("platoon_predecessor", 1.0, range_m=40.0, relative_speed_mps=0.0)
        self.assertNotEqual(
            self.tracker.state("cut_in").estimated_range_m,
            self.tracker.state("platoon_predecessor").estimated_range_m,
        )
        old = self.tracker.update("cut_in", 0.5, range_m=5.0)
        self.assertEqual(old.status, "invalid_timestamp")

    def test_missing_target_is_explicit(self) -> None:
        state = self.tracker.predict_missing("never_seen", 1.0)
        self.assertEqual(state.status, "uninitialized_missing_radar")
        self.assertIsNone(state.estimated_range_m)


class V2XCompatibilityTests(unittest.TestCase):
    def test_delayed_message_is_propagated_not_current(self) -> None:
        message = V2XMessage("truck_0", 1.0, 20.0, -2.0, True, "truck_1", 1)
        observation = propagate_predecessor_v2x(message, 1.5, stale_after_s=1.0)
        self.assertEqual(observation.status, "fresh_propagated")
        self.assertAlmostEqual(observation.message_age_s, 0.5)
        self.assertAlmostEqual(observation.propagated_speed_mps, 19.0)
        self.assertGreater(observation.speed_variance, 1.0)

    def test_stale_message_is_not_current_and_identity_remains_separate(self) -> None:
        message = V2XMessage("platoon_predecessor", 1.0, 20.0, 0.0, False, "ego", 3)
        stale = propagate_predecessor_v2x(message, 2.5, stale_after_s=1.0)
        self.assertEqual(stale.status, "stale")
        tracker = RadarTargetTracker()
        radar = tracker.update("cut_in", 2.5, range_m=15.0, relative_speed_mps=3.0)
        self.assertEqual(stale.sender_id, "platoon_predecessor")
        self.assertEqual(radar.target_vehicle_id, "cut_in")

    def test_estimator_neither_validates_nor_duplicates_pending_channel(self) -> None:
        channel = PendingV2XChannel(("leader", "follower"), {"leader": "follower"}, seed=3)
        message = V2XMessage("leader", 1.0, 12.0, 0.0, False, "follower", 1)
        channel.enqueue(message, latency_ms=100.0)
        delivered, _ = channel.deliver_ready(1.05)
        self.assertEqual(delivered, [])
        delivered, _ = channel.deliver_ready(1.1)
        observation = propagate_predecessor_v2x(delivered[0], 1.1, stale_after_s=1.0)
        self.assertEqual(observation.status, "fresh_propagated")

    def test_acceleration_process_variance_rejects_negative_and_nonfinite_values(self) -> None:
        message = V2XMessage("leader", 1.0, 12.0, 0.0, False, "follower", 1)
        for invalid in (-0.1, float("nan"), float("inf"), float("-inf")):
            with self.subTest(acceleration_process_variance=invalid):
                with self.assertRaisesRegex(ValueError, "finite and non-negative"):
                    propagate_predecessor_v2x(
                        message, 1.5, stale_after_s=1.0, acceleration_process_variance=invalid
                    )

    def test_acceleration_process_variance_allows_zero_boundary(self) -> None:
        message = V2XMessage("leader", 1.0, 12.0, 0.0, False, "follower", 1)
        observation = propagate_predecessor_v2x(
            message,
            1.5,
            stale_after_s=1.0,
            speed_variance=2.0,
            acceleration_variance=3.0,
            acceleration_process_variance=0.0,
        )
        self.assertEqual(observation.status, "fresh_propagated")
        self.assertEqual(observation.speed_variance, 2.0)
        self.assertEqual(observation.acceleration_variance, 3.0)


class BoundaryTests(unittest.TestCase):
    def test_estimator_has_no_simulator_or_evaluation_import(self) -> None:
        source = (Path(__file__).parents[1] / "samtruck" / "estimation.py").read_text(encoding="utf-8").lower()
        for forbidden in ("import traci", "import sumolib", "from traci", "from sumolib", "evaluation logger"):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
