from dataclasses import dataclass


@dataclass(frozen=True)
class BinaryClassificationMetrics:
    true_positive: int
    false_positive: int
    true_negative: int
    false_negative: int
    sensitivity: float
    specificity: float
    precision: float
    accuracy: float
    f1_score: float
