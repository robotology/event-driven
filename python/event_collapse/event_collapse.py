import re
import numpy as np
import argparse
import os


def pretty_print_AE(bottlenumber, timestamp, events):
    return '%d %f AE (%s)' %(bottlenumber, timestamp,
                             ' '.join(map(str, list(events))))


def collapse(filename, min_bot_duration=2e-3):

    pattern = re.compile('(\d+) (\d+\.\d+) AE \((.*)\)')

    with open(filename, 'r') as f:
        content = f.read()
        found = pattern.findall(content)
        buffer = []
        first_timestamp = None
        perc_done = 0
        for n, elem in enumerate(found):

            events = np.int32(elem[2].split())
            bottlenumber = np.int32(elem[0])
            timestamp = np.float64(elem[1])
            buffer.append(events)
            if first_timestamp is None:
                first_timestamp = timestamp
            if timestamp - first_timestamp > min_bot_duration:
                first_timestamp = timestamp
                pp = pretty_print_AE(bottlenumber, timestamp,
                                      np.concatenate(buffer))
                buffer = []
                yield pp
            if n / len(found) > perc_done + 0.05:
                perc_done += 0.05
                print("{}% done".format(int(perc_done * 100)))



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Collapse events in bottles of duration at least min_bot_duration milliseconds')
    parser.add_argument('in_file', type=str, help='Input log file')
    parser.add_argument('--min_bot_duration', '-m', default=2, type=int, nargs=1, help='Minimum duration of bottle into which events are collapsed')
    parser.add_argument('--out_file', type=str, help='Output log file')

    args = parser.parse_args()

    if args.out_file is None:
        args.out_file = os.path.join(os.path.dirname(args.in_file), 'data_collapsed.log')

    text_out = '\n'.join([line for line in collapse(args.in_file, args.min_bot_duration * 1e-3)])
    with open(args.out_file, 'w') as of:
        of.write(text_out)
    print("Written to {}".format(args.out_file))
