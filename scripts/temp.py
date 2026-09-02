#template for file conversion between txt to csv: 

import csv
from pathlib import Path


def convert_txt_to_csv(
    input_txt_file: str, output_csv_file: str, parse_line_func
):
    """Reads a raw text file line-by-line, parses it, and exports it to a standard CSV file."""
    input_path = Path(input_txt_file)
    output_path = Path(output_csv_file)

    print(f"Converting: {input_path.name} -> {output_path.name}")

    # Ensure output directory exists
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(input_path, "r") as infile, open(
        output_path, "w", newline=""
    ) as outfile:
        # Configure standard comma-separated writing
        writer = csv.writer(outfile, delimiter=",")

        # Write standard trajectory header
        writer.writerow(
            ["timestamp", "px", "py", "pz", "qx", "qy", "qz", "qw"]
        )

        for line in infile:
            # Drop empty rows or text comment markers (#)
            if not line.strip() or line.startswith("#"):
                continue

            try:
                # Extract numerical list using custom parse logic
                parsed_data = parse_line_func(line)
                if parsed_data:
                    writer.writerow(parsed_data)
            except Exception as e:
                print(f"Skipping corrupt row in {input_path.name}: {e}")


# ==================================================
# CUSTOM PARSER RULES (Update based on your txt data)
# ==================================================


def parse_groundtruth_txt(line: str):
    """Example parser for groundtruth text split by spaces or tabs."""
    # .split() handles arbitrary spaces or tab delimiters automatically
    parts = line.strip().split()

    # Convert strings to floats
    # Expected: time, x, y, z, qx, qy, qz, qw
    return [float(x) for x in parts[0:8]]


def parse_openvins_txt(line: str):
    """Example parser for OpenVINS text logs."""
    parts = line.strip().split()

    # Adapt indices if your openvins text log output maps columns differently
    timestamp = float(parts[0])
    px, py, pz = float(parts[1]), float(parts[2]), float(parts[3])
    qx, qy, qz, qw = (
        float(parts[4]),
        float(parts[5]),
        float(parts[6]),
        float(parts[7]),
    )

    return [timestamp, px, py, pz, qx, qy, qz, qw]


# ==================================================
# EXECUTION
# ==================================================
if __name__ == "__main__":
    # Convert your groundtruth text logs to CSV
    convert_txt_to_csv(
        input_txt_file="groundtruth.txt",
        output_csv_file="groundtruth.csv",
        parse_line_func=parse_groundtruth_txt,
    )

    # Convert your OpenVINS text logs to CSV
    convert_txt_to_csv(
        input_txt_file="openvins_output.txt",
        output_csv_file="openvins_output.csv",
        parse_line_func=parse_openvins_txt,
    )

Key HighlightsDelimiter Shift: Changes delimiter=" " to standard delimiter=","
for valid CSV outputs.Header Support: Adds explicit column naming
(timestamp, px, etc.) to make the CSVs easily readable by tools like Pandas
or Excel.Delimiter Agnostic Inputs: The .split() call inside the parsing 
methods cleans up multiple spaces, tabs,or single spaces commonly found in raw text data logs.