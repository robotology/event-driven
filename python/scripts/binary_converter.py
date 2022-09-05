from bimvee.importIitYarp import importIitYarpBinaryDataLog
from bimvee.exportIitYarp import exportIitYarp
import argparse
from os.path import getsize

parser = argparse.ArgumentParser(description='Converter from binary to yarp format')
parser.add_argument('--input', '-i', dest='input_path', type=str, required=True,
                    help='Path to input file')
parser.add_argument('--output', '-o', dest='output_path', type=str, required=True,
                    help='Path to output file')

args = parser.parse_args()

maxBytesToImport = 320000000

for i in range(0, getsize(args.input_path), maxBytesToImport):
    data = importIitYarpBinaryDataLog(filePathOrName=args.input_path,
                                      importMaxBytes=maxBytesToImport,
                                      importFromByte=i,
                                      zeroTimestamps=False
                                      )
    exportIitYarp(data, exportFilePath=args.output_path, writeMode='a', protectedWrite=False, exportTimestamps=False)