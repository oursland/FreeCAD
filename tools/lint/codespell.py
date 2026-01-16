#!/usr/bin/env python3
import argparse
import sys
import os
import urllib.request
import logging

from utils import (
    run_command,
    init_environment,
    write_file,
    append_file,
    emit_problem_matchers,
)


def download_dictionary(url: str, dest: str) -> bool:
    logging.info(f"Downloading dictionary from {url} to {dest}")
    try:
        urllib.request.urlretrieve(url, dest)
        return True
    except Exception as e:
        logging.warning(f"Error downloading dictionary: {e}")
        return False



def generate_markdown_report(misspellings: int, log_file: str) -> str:
    """
    Generate a Markdown report section based on the codespell results and log file.
    """
    report_lines = []
    if misspellings > 0:
        report_lines.append(
            f"<details><summary>:pencil2: Codespell found {misspellings} misspellings</summary>"
        )
    else:
        report_lines.append(
            "<details><summary>:heavy_check_mark: Codespell found no misspellings</summary>"
        )
    report_lines.append("")
    report_lines.append(
        "To ignore false positives, append the word to the [.github/codespellignore]"
        "(https://github.com/FreeCAD/FreeCAD/blob/master/.github/codespellignore) file (lowercase)"
    )
    report_lines.append("````")
    with open(log_file, "r", encoding="utf-8") as lf:
        report_lines.append(lf.read())
    report_lines.append("````")
    report_lines.append("</details>")
    report_lines.append("")
    return "\n".join(report_lines)


def main():
    parser = argparse.ArgumentParser(
        description="Run codespell on files and append a Markdown report."
    )
    parser.add_argument(
        "--files",
        action="extend",
        nargs="+",
        required=True,
        help="List of files to spell check."
    )
    parser.add_argument(
        "--ignore-words",
        required=True,
        help="Comma-separated list of words to ignore (from codespellignore).",
    )
    parser.add_argument(
        "--log-dir",
        required=True,
        help="Directory to store the log file."
    )
    parser.add_argument(
        "--report-file",
        required=True,
        help="Name of the report file."
    )
    parser.add_argument(
        "--skip", required=True, help="Comma-separated list of file patterns to skip."
    )
    parser.add_argument(
        "--verbose",
        action='store_true',
        help="Use verbose output."
    )
    args = parser.parse_args()
    init_environment(args)
    dictionary_file = os.path.join(args.log_dir, 'codespell_dictionary.txt')
    use_dictionary = os.path.exists(dictionary_file)
    if not use_dictionary:
        dictionary_url = 'https://raw.githubusercontent.com/codespell-project/codespell/master/codespell_lib/data/dictionary.txt'
        logging.info('Dictionary not found; downloading...')
        use_dictionary = download_dictionary(dictionary_url, dictionary_file)

    run_command([sys.executable, '-m', 'pip', 'install', '-q', 'codespell'], check=True)
    cmd = [
        sys.executable,
        '-m',
        'codespell',
        '--quiet-level',
        '3',
        '--summary',
        '--count',
        '--ignore-words',
        args.ignore_words,
        '--skip',
        args.skip,
    ]
    if use_dictionary and os.path.exists(dictionary_file):
        cmd += ['-D', dictionary_file]
    cmd += args.files
    stdout, stderr, _ = run_command(cmd)
    output = stdout + "\n" + stderr + "\n"

    log_file = os.path.join(args.log_dir, "codespell.log")
    write_file(log_file, output)
    emit_problem_matchers(log_file, "codespell.json", "codespell")
    misspellings: int | None
    try:
        misspellings = int(stdout.strip())
    except ValueError:
        logging.error(f"Could not parse misspellings count from output:
{stdout}")
        misspellings = None

    logging.info(f"Found {misspellings} misspellings")

    report = generate_markdown_report(misspellings, log_file)
    append_file(args.report_file, report + "\n")

    if misspellings is None:
        sys.exit(1)
    sys.exit(0 if misspellings == 0 else 1)


if __name__ == "__main__":
    main()
