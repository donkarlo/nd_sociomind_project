from anomaly_graphing.data.model.binary_classification_metrics import BinaryClassificationMetrics


class BinaryClassificationEvaluator:
    def evaluate(self, values: tuple[float, ...], threshold: float, ground_truth: tuple[bool, ...]) -> BinaryClassificationMetrics:
        if len(values) != len(ground_truth):
            raise ValueError("Prediction values and ground-truth labels must have the same length.")

        true_positive = 0
        false_positive = 0
        true_negative = 0
        false_negative = 0

        for value, actual_positive in zip(values, ground_truth):
            predicted_positive = value > threshold

            if predicted_positive and actual_positive:
                true_positive += 1
            elif predicted_positive and not actual_positive:
                false_positive += 1
            elif not predicted_positive and not actual_positive:
                true_negative += 1
            else:
                false_negative += 1

        sensitivity = self._safe_divide(true_positive, true_positive + false_negative)
        specificity = self._safe_divide(true_negative, true_negative + false_positive)
        precision = self._safe_divide(true_positive, true_positive + false_positive)
        accuracy = self._safe_divide(true_positive + true_negative, len(values))
        f1_score = self._safe_divide(2 * precision * sensitivity, precision + sensitivity)

        return BinaryClassificationMetrics(
            true_positive,
            false_positive,
            true_negative,
            false_negative,
            sensitivity,
            specificity,
            precision,
            accuracy,
            f1_score,
        )

    def _safe_divide(self, numerator: float, denominator: float) -> float:
        if denominator == 0:
            return 0.0
        return numerator / denominator
