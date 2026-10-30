import math


class GaussianWassersteinDistanceCalculator:
    def calculate_diagonal(self, first_mean: tuple[float, ...], first_standard_deviation: tuple[float, ...], second_mean: tuple[float, ...], second_standard_deviation: tuple[float, ...]) -> float:
        if len(first_mean) != len(second_mean):
            raise ValueError("Gaussian means must have the same dimension.")
        if len(first_standard_deviation) != len(second_standard_deviation):
            raise ValueError("Gaussian standard deviations must have the same dimension.")
        if len(first_mean) != len(first_standard_deviation):
            raise ValueError("Gaussian mean and standard-deviation dimensions must match.")

        squared_distance = 0.0
        for index in range(len(first_mean)):
            mean_difference = first_mean[index] - second_mean[index]
            standard_deviation_difference = first_standard_deviation[index] - second_standard_deviation[index]
            squared_distance += mean_difference * mean_difference
            squared_distance += standard_deviation_difference * standard_deviation_difference

        return math.sqrt(squared_distance)
