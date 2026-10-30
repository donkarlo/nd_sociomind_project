import math


class RobustAnomalyNormalizer:
    def normalize(self, values: list[float]) -> tuple[float, ...]:
        if not values:
            return tuple()

        sorted_values = sorted(values)
        baseline_limit_index = max(1, int(0.60 * len(sorted_values)))
        baseline_values = sorted_values[:baseline_limit_index]
        baseline = self._median(baseline_values)
        absolute_deviations = [abs(value - baseline) for value in baseline_values]
        median_absolute_deviation = self._median(absolute_deviations)
        robust_scale = 1.4826 * median_absolute_deviation
        fallback_scale = max(1e-9, 0.03 * max(sorted_values[-1], 1e-9))
        scale = max(robust_scale, fallback_scale)

        normalized: list[float] = []
        for value in values:
            positive_deviation = max(0.0, value - baseline)
            score = 1.0 - math.exp(-positive_deviation / (2.5 * scale))
            normalized.append(min(1.0, max(0.0, score)))
        return tuple(normalized)

    def _median(self, values: list[float]) -> float:
        if not values:
            return 0.0
        ordered = sorted(values)
        middle = len(ordered) // 2
        if len(ordered) % 2 == 1:
            return ordered[middle]
        return 0.5 * (ordered[middle - 1] + ordered[middle])
