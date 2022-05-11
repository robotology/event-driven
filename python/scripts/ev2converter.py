from bimvee.importIitYarp import importIitYarp
from bimvee.exportIitYarp import exportIitYarp
import argparse
import os, shutil

parser = argparse.ArgumentParser()
parser.add_argument('--in_path',  dest='in_path', type=str, required=True, help='Path to input file to be converted')
parser.add_argument('--out_path', dest='out_path', type=str, nargs=1, help='Path to save converted file to')
args = parser.parse_args()

if args.out_path is None:
    if os.path.isdir(args.in_path):
        args.out_path = os.path.join(args.in_path, 'ev2')
    else:
        args.out_path = os.path.join(os.path.dirname(args.in_path), 'ev2')

if os.path.exists(args.out_path):
    if input(
            f'Directory {args.out_path} already exists. Are you sure you want to replace all of its content? (y/n)\n') == 'y':
        shutil.rmtree(args.out_path)
    else:
        print('Exiting')
        exit(0)

data = importIitYarp(filePathOrName=args.in_path)
exportIitYarp(data, exportFilePath=args.out_path, exportAsEv2=True)

