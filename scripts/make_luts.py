# -*- coding: utf-8 -*-
"""
    Script to create PLL Look Up Tables for QMAC HF-90 free-tune conversion software for Teensy 4.0
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


# TODO: handle special points
# TODO: add LO2 index offsets to correct LO1 error

from pathlib import Path
import pandas as pd


LO1_PRESC = 32
LO2_PRESC = 64


# open excel files

LO1_filename = "LO1_results.csv"
LO2_filename = "LO2_results.csv"


LO1data = pd.read_csv(LO1_filename)
LO1_NTOT = LO1data["N"]

LO1_N = [int(item / LO1_PRESC) for item in LO1_NTOT]
LO1_A = [int(item % LO1_PRESC) for item in LO1_NTOT]
LO1_R = LO1data["R"]

F_LO1 = LO1data["f_wanted"]

LO2data = pd.read_csv(LO2_filename)
LO2_NTOT = LO2data["N"]

LO2_N = [int(item / LO2_PRESC) for item in LO2_NTOT]
LO2_A = [int(item % LO2_PRESC) for item in LO2_NTOT]
LO2_R = LO2data["R"]

F_LO2 = list(LO2data["f_wanted"])

mid_idx = F_LO2.index(83616000)
print(f"LO2 middle index: {mid_idx}")

# get stepsizes
LO1_STEPSIZE = int(F_LO1[1] - F_LO1[0])
LO2_STEPSIZE = int(F_LO2[1] - F_LO2[0])


# TODO @@@ take errors and calculate correction offset index for LO2

LO1_error = LO1data["error"]
LO2_correction = [round(LO1_e / LO2_STEPSIZE) for LO1_e in LO1_error]
print(LO2_correction)



# write to header file
out = ""
out += "/* \n Synthesizer Look Up Tables\n */"
out += "\n\n"


out += (
    "const int LO2_MID_IDX = "
    + str(mid_idx)
    + "; // the index in the LO2 LUT that corresponds to 83616000 Hz\n"
)
out += "const int LO1_STEPSIZE = " + str(LO1_STEPSIZE) + "; // Hz\n"
out += "const int LO2_STEPSIZE = " + str(LO2_STEPSIZE) + "; // Hz\n"


out += "/* \n LO1 / SUB N\n */"

# print in C++ format
out += "#define LO1_NPTS " + str(len(LO1_N)) + " \n \n"
out += "const short Ns_LUT[LO1_NPTS] = {\n"

for item in LO1_N:
    out += str(item)
    out += ","
    out += "\n"

out += "};"


out += "\n\n"

out += "/* \n LO1 / SUB A\n */"

# print in C++ format
out += "const short As_LUT[LO1_NPTS] = {\n"

for item in LO1_A:
    out += str(item)
    out += ","
    out += "\n"

out += "};"


out += "\n\n"



out += "/* \n LO1 / SUB R\n */"

# print in C++ format
out += "const short Rs_LUT[LO1_NPTS] = {\n"

for item in LO1_R:
    out += str(item)
    out += ","
    out += "\n"

out += "};"


out += "\n\n"

out += "/* \n LO1 / SUB error correction (corrected on LO2)\n */"
out += "const short Cor_LUT[LO1_NPTS] = {\n"

for item in LO2_correction:
    out += str(item)
    out += ","
    out += "\n"

out += "};"


out += "\n\n"

out += "/* \n LO2 / MAIN N\n */"
# print in C++ format
out += "#define LO2_NPTS " + str(len(LO2_N)) + " \n \n"
out += "const short N_LUT[LO2_NPTS] = {\n"

for item in LO2_N:
    out += str(item)
    out += ","
    out += "\n"

out += "};"


out += "\n\n"

out += "/* \n LO2 / MAIN A\n */"

# print in C++ format
out += "const short A_LUT[LO2_NPTS] = {\n"

for item in LO2_A:
    out += str(item)
    out += ","
    out += "\n"

out += "};"

out += "\n\n"
out += "/* \n LO2 / MAIN R\n */"

# print in C++ format
out += "const short R_LUT[LO2_NPTS] = {\n"

for item in LO2_R:
    out += str(item)
    out += ","
    out += "\n"

out += "};"

# get the path to the current working directory

filepath = Path.cwd().parent / "src" / "free_hf_90"


text_file = open(filepath / "Luts.h", "w")
n = text_file.write(out)
text_file.close()
