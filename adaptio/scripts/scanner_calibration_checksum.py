import argparse  # For command-line argument parsing
import hashlib
import os
import sys


def get_and_calculate_file_checksum(filepath):
    """
    Calculates the MD5 checksum of a YAML file, excluding the 'checksum' field.

    Args:
        filepath (str): The path to the YAML file.

    Returns:
        str: The calculated MD5 checksum, or None if the file cannot be read or parsed.
    """

    checksum = ""
    try:
        data = ""
        for ln in open(filepath, "r"):
            if ln.startswith("checksum"):
                fields = ln.split()
                if len(fields) > 1:
                    checksum = fields[1]
                break
            data += ln

        md5_hash = hashlib.md5(data.encode("utf-8")).hexdigest()
    except FileNotFoundError:
        print(f"Error: File not found at {filepath}")
        return None

    return (md5_hash, checksum)


def process_path(filepath, command):
    """
    Calculate or check the checksum for a YAML file.

    Args:
        filepath (str): The path to the YAML file.
        command (str): The coommand, either 'check' or 'calculate'.

    Returns:
        bool: False if command='check' and the checksum does not match, else True.
    """

    (calculated_checksum, current_checksum) = get_and_calculate_file_checksum(filepath)

    ok = True
    if command == "check":
        print(f"file: {filepath}")
        print(f"  current:    {current_checksum}")
        print(f"  calculated: {calculated_checksum}")
        print(
            f"  equal:      {'yes' if current_checksum == calculated_checksum else 'no'}"
        )

        ok = current_checksum == calculated_checksum

    if command == "calculate":
        print(f"file: {filepath}")
        print(f"  calculated: {calculated_checksum}")

    return ok


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Calculate or check MD5 checksums for YAML files, excluding the 'checksum' field."
    )
    parser.add_argument(
        "command",
        choices=["check", "calculate"],
        help="Action to perform: 'check' existing checksums or 'calculate' and update them.",
    )
    parser.add_argument(
        "path", help="Path to a YAML file or a directory containing YAML files."
    )

    args = parser.parse_args()

    success = True
    if os.path.isfile(args.path):
        success = process_path(args.path, args.command)
    elif os.path.isdir(args.path):
        for root, _, files in os.walk(args.path):
            for indx, file in enumerate(files):
                if file.lower().endswith(".yaml"):
                    if indx > 0:
                        print()

                    filepath = os.path.join(root, file)
                    ok = process_path(filepath, args.command)
                    success = ok if success else success
    else:
        print(f"Error: Path '{args.path}' is neither a file nor a directory.")
        success = False

    sys.exit(0 if success else -1)
