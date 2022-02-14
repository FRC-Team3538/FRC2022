import csv
import argparse
from collections import defaultdict
from timeit import default_timer as timer
import bisect


def binarySearch(alist, item, key):
    first = 0
    last = len(alist) - 1
    found = False

    while first <= last and not found:
        pos = 0
        midpoint = (first + last) // 2
        val = key(alist[midpoint])
        if val == item:
            pos = midpoint
            found = True
        else:
            if item < val:
                # print(f'{item} < {val}, searching before {midpoint}')
                last = midpoint - 1
            else:
                # print(f'{item} > {val}, searching after {midpoint}')
                first = midpoint + 1
    if not found:
        # first_key = key(alist[first])
        # last_key = key(alist[last])
        # print(f'found these two: {first_key} and {last_key}')
        return (first, found)
    return (pos, found)


def read_csv(args):
    with open(args.input, newline="", buffering=1) as f:
        reader = csv.DictReader(f)
        ranges = []

        candidate_rows = []
        rows = []

        active_window = False
        was_active_window = False
        min_time = 0
        max_time = 0
        keys = reader.fieldnames

        for row in reader:
            # if on_condition, add all rows after before_window to actual rows
            # then just add row to rows until off_condition
            # then set max timestamp and add to `rows` until that condition is met or until on_condition returns true
            time = float(row["timestamp"])

            if eval(args.begin_condition) and not active_window:
                print("on_condition returned true at", time)

                if not was_active_window:
                    min_time = max(0, time - args.before)

                    candidate_min_time = candidate_rows[0]['timestamp']
                    candidate_max_time = candidate_rows[-1]['timestamp']

                    if min_time < float(candidate_min_time):
                        rows.extend(candidate_rows)
                    elif min_time > float(candidate_max_time):
                        pass
                    else:
                        # it's somewhere in between
                        print(f'binary searching for {min_time} between {candidate_min_time} and {candidate_max_time}')

                        (candidate_min_index, _) = binarySearch(
                            candidate_rows,
                            min_time,
                            lambda row: float(row["timestamp"]),
                        )

                        rows.extend(candidate_rows[candidate_min_index:])
                        
                    rows.append(row)

                    del candidate_rows
                    candidate_rows = []

                active_window = True
                was_active_window = False

            elif eval(args.end_condition) and active_window:
                print("off_condition returned true at", time)
                active_window = False
                was_active_window = True
                max_time = time + args.after

            if active_window or time < max_time:
                rows.append(row)
            else:
                candidate_rows.append(row)

            if was_active_window and time > max_time:
                print(f"adding range of {min_time}-{max_time}")
                write_file(args.output, rows, keys)
                del(rows)
                rows = []
                was_active_window = False

    if len(rows) > 0:
        print(f"adding range of {min_time}-{max_time}")
        write_file(args.output, rows, keys)
        del(rows)
    del(candidate_rows)
    return (rows, keys, ranges)


def write_file(output, rows, keys):
    print("writing rows:", len(rows))

    fname = output + "_" + rows[0]["timestamp"] + "-" + rows[-1]["timestamp"] + ".csv"

    with open(fname, "w", newline="", buffering=1) as f:
        print(f"writing file {fname}")
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
        f.flush()
    print("write complete")


if __name__ == "__main__":
    start = timer()
    parser = argparse.ArgumentParser(
        description="Extract Enable Periods from wpilog CSV exports. Multiple overlapping windows are treated as one."
    )
    parser.add_argument("-i", "--input", help="input CSV file representing a WPILog")
    parser.add_argument(
        "-o", "--output", help="output filename to write filtered results to"
    )
    parser.add_argument(
        "-B", "--before", help="seconds before enable period to include", default=10
    )
    parser.add_argument(
        "-A", "--after", help="seconds after enable period to include", default=10
    )
    parser.add_argument(
        "-m",
        "--multiple",
        action="store_true",
        help="output one file per range instead of one total. NOT IMPLEMENTED",
    )
    parser.add_argument(
        "-b",
        "--begin-condition",
        default="row['DS:enabled'] == '1'",
        help="when this is true, begin window",
    )
    parser.add_argument(
        "-e",
        "--end-condition",
        default="row['DS:enabled'] == '0'",
        help="when this is true, end window",
    )

    args = parser.parse_args()
    args.before = int(args.before)
    args.after = int(args.after)

    if args.output[-4:] == ".csv":
        args.output = args.output[:-4]

    keys = []

    (rows, keys, ranges) = read_csv(args)
    print(f"completed in {timer() - start} seconds")
