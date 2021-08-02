from bimvee.importIitYarp import importIitYarpBinaryDataLog
from bimvee.exportIitYarp import exportIitYarp
import argparse

parser = argparse.ArgumentParser(description='Converter from binary to yarp format')
parser.add_argument('--input', '-i', dest='input_path', type=str, required=True,
                    help='Path to input file')
parser.add_argument('--output', '-o', dest='output_path', type=str, required=True,
                    help='Path to output file')

args = parser.parse_args()

data = importIitYarpBinaryDataLog(filePathOrName=args.input_path)
exportIitYarp(data, exportFilePath=args.output_path)