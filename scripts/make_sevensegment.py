# -*- coding: utf-8 -*-
"""
    Script to create seven segment Look Up Tables for QMAC HF-90 free-tune conversion software for Teensy 4.0
    Copyright (C) 2024 Wouter Jan Ubbels PE4WJ
    pe4wj@amsat.org

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""


from pathlib import Path
import pandas as pd


# 0b11110110, //0

# open excel files

digit_filename = "seven_segment.csv"
data = pd.read_csv(digit_filename)

out = ""
out += "/* \n Seven segment Look Up Tables\n */"
out += "\n\n"


out += "/* \n use an array of structs in a dict-type fashion to look up 7 segment display segments for each alphanumeric input character\n */\n"
out += "typedef struct { // frequency memory \n"
out += "String alphanum; // corresponding alphanumeric character \n"
out += "uint8_t segments; // segments which are lit \n"
out += "} segdict; \n\n"

nalphas = len(data["alpha"])
out += "const uint8_t nalphas = " + str(nalphas) + ";\n\n"


# out += "#define LO1_NPTS " + str(len(LO1_N)) +" \n \n"
out += "const segdict lookup_segments[] = {\n"

segmap = ["b", "f", "g", "a", "c", "d", "e"]

for alpha, segments in zip(data["alpha"], data["used_segments"]):
    o = 0
    for c in segments:

        try:

            i = segmap.index(c)
            o += 2 ** (i + 1)
        except ValueError:
            print("character not in segmap")
            o = 0  # blank digit
    # create table row
    row = "{" + '"' + str(alpha) + '"' + ", " + str(o) + "},\n"
    out += row
out += "};"

print(out)

filepath = Path.cwd().parent / "src" / "free_hf_90"


text_file = open(filepath / "Seven_segment.h", "w")
n = text_file.write(out)
text_file.close()
