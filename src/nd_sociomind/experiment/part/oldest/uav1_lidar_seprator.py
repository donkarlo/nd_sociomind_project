from pathlib import Path
import zipfile
import numpy
from numpy.lib.format import open_memmap


class RosLaserScanYamlToNpzExporter:
    def __init__(self, input_file_path: str, output_file_path: str, lidar_dimension: int = 720,
                 keep_infinity: bool = True, compressed: bool = False):
        self.input_file_path = Path(input_file_path)
        self.output_file_path = Path(output_file_path)
        self.lidar_dimension = lidar_dimension
        self.keep_infinity = keep_infinity
        self.compressed = compressed
        self.temporary_npy_file_path = self.output_file_path.with_suffix(".temporary_data.npy")

    def export(self) -> None:
        if not self.input_file_path.exists():
            raise FileNotFoundError(f"Input file does not exist: {self.input_file_path}")

        row_count = self.count_rows()

        if row_count == 0:
            raise ValueError("No valid LiDAR scans were found.")

        if self.temporary_npy_file_path.exists():
            self.temporary_npy_file_path.unlink()

        data = open_memmap(
            self.temporary_npy_file_path,
            mode="w+",
            dtype=numpy.float64,
            shape=(row_count, self.lidar_dimension + 1)
        )

        row_index = 0

        for row in self.iter_lidar_rows():
            data[row_index, :] = row
            row_index += 1

        data.flush()
        del data

        self.write_npz_from_npy_file()

        if self.temporary_npy_file_path.exists():
            self.temporary_npy_file_path.unlink()

        print(f"Saved: {self.output_file_path}")
        print(f"Rows: {row_count}")
        print(f"Shape: ({row_count}, {self.lidar_dimension + 1})")

    def count_rows(self) -> int:
        row_count = 0

        for row in self.iter_lidar_rows():
            row_count += 1

        return row_count

    def iter_lidar_rows(self):
        current_seconds = None
        current_nanoseconds = None

        inside_bracket_ranges = False
        bracket_ranges_parts = []

        inside_list_ranges = False
        list_ranges_values = []

        with self.input_file_path.open("r", encoding="utf-8", errors="ignore") as input_file:
            for line in input_file:
                stripped_line = line.strip()

                if stripped_line == "":
                    continue

                if inside_bracket_ranges:
                    bracket_ranges_parts.append(stripped_line)

                    if "]" in stripped_line:
                        ranges = self.parse_bracket_ranges(" ".join(bracket_ranges_parts))
                        row = self.build_row(current_seconds, current_nanoseconds, ranges)

                        if row is not None:
                            yield row

                        inside_bracket_ranges = False
                        bracket_ranges_parts = []

                    continue

                if inside_list_ranges:
                    if stripped_line.startswith("-"):
                        value_text = stripped_line[1:].strip()
                        list_ranges_values.append(self.parse_float(value_text))

                        if len(list_ranges_values) == self.lidar_dimension:
                            row = self.build_row(current_seconds, current_nanoseconds, list_ranges_values)

                            if row is not None:
                                yield row

                            inside_list_ranges = False
                            list_ranges_values = []

                        continue
                    else:
                        inside_list_ranges = False
                        list_ranges_values = []

                if stripped_line.startswith("secs:"):
                    current_seconds = self.parse_integer_after_colon(stripped_line)
                    continue

                if stripped_line.startswith("nsecs:"):
                    current_nanoseconds = self.parse_integer_after_colon(stripped_line)
                    continue

                if stripped_line.startswith("ranges:"):
                    ranges_text = stripped_line.split(":", 1)[1].strip()

                    if ranges_text.startswith("["):
                        bracket_ranges_parts = [ranges_text]

                        if "]" in ranges_text:
                            ranges = self.parse_bracket_ranges(ranges_text)
                            row = self.build_row(current_seconds, current_nanoseconds, ranges)

                            if row is not None:
                                yield row

                            bracket_ranges_parts = []
                        else:
                            inside_bracket_ranges = True

                    else:
                        inside_list_ranges = True
                        list_ranges_values = []

    def parse_integer_after_colon(self, line: str) -> int:
        value_text = line.split(":", 1)[1].strip()
        return int(value_text)

    def parse_bracket_ranges(self, ranges_text: str) -> numpy.ndarray:
        start_index = ranges_text.find("[")
        end_index = ranges_text.find("]")

        if start_index == -1 or end_index == -1:
            return numpy.asarray([], dtype=numpy.float64)

        values_text = ranges_text[start_index + 1:end_index]
        values = numpy.fromstring(values_text, sep=",", dtype=numpy.float64)

        return values

    def parse_float(self, value_text: str) -> float:
        lowered_value_text = value_text.lower()

        if lowered_value_text == "inf" or lowered_value_text == "+inf" or lowered_value_text == ".inf" or lowered_value_text == "+.inf":
            return numpy.inf

        if lowered_value_text == "-inf" or lowered_value_text == "-.inf":
            return -numpy.inf

        if lowered_value_text == "nan" or lowered_value_text == ".nan":
            return numpy.nan

        return float(value_text)

    def build_row(self, seconds: int | None, nanoseconds: int | None, ranges) -> numpy.ndarray | None:
        if seconds is None:
            return None

        if nanoseconds is None:
            return None

        if len(ranges) != self.lidar_dimension:
            return None

        row = numpy.empty(self.lidar_dimension + 1, dtype=numpy.float64)

        timestamp = float(seconds) + float(nanoseconds) * 1e-9
        row[0] = timestamp
        row[1:] = ranges

        if not self.keep_infinity:
            infinity_mask = numpy.isinf(row)
            row[infinity_mask] = -1.0

        return row

    def write_npz_from_npy_file(self) -> None:
        if self.output_file_path.exists():
            self.output_file_path.unlink()

        if self.compressed:
            compression_method = zipfile.ZIP_DEFLATED
        else:
            compression_method = zipfile.ZIP_STORED

        with zipfile.ZipFile(self.output_file_path, mode="w", compression=compression_method,
                             compresslevel=1) as zip_file:
            zip_file.write(self.temporary_npy_file_path, arcname="data.npy")


if __name__ == "__main__":
    input_file_path = "/home/donkarlo/Dropbox/repo/nd_sociomind_project/src/nd_sociomind/experiment/part/oldest/uav1_normal_episode.yaml"
    output_file_path = "/home/donkarlo/Dropbox/repo/nd_sociomind_project/src/nd_sociomind/experiment/part/oldest/uav1_lidar_normal.npz"

    exporter = RosLaserScanYamlToNpzExporter(
        input_file_path=input_file_path,
        output_file_path=output_file_path,
        lidar_dimension=720,
        keep_infinity=True,
        compressed=False
    )

    exporter.export()