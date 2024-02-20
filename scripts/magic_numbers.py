# -*- coding: utf-8 -*-
"""
    Script to calculate PLL "magic numbers" for QMAC HF-90 free-tune conversion software for Teensy 4.0
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

import numpy as np
from tqdm import tqdm
import pandas as pd


# Input parameters
F_REF = 24_000_000  # master reference frequency
LO1_STEP = 20000  # Hz frequency step of LO1
LO2_STEP = 25  # Hz frequency step of LO2

ALLOWED_ERROR_LO1 = (
    3000
)
ALLOWED_ERROR_LO1_GAPS = 3000 # Hz max allowed frequency error of LO2 in frequencies which cannot fulfill the nominal allowed error  # Hz max allowed frequency error of LO1, this error needs to be corrected by offsetting LO2

ALLOWED_ERROR_LO2 = 15#12.5  # Hz max allowed frequency error of LO2
ALLOWED_ERROR_LO2_GAPS = 50#12.5  # Hz max allowed frequency error of LO2 in frequencies which cannot fulfill the nominal allowed error

F_LO2_CENTER = 83616000  # Hz Center frequency of LO2
SIDEBANDMARGIN = (
    2400
)  # Hz margin to allow sideband (BFO) correction to be carried out using LO2

# min and max phase comparison frequencies per loop
FR_MIN_LO1 = 40_000
FR_MIN_LO1_GAPS = 3_000

FR_MAX_LO1 = 202_000

FR_MIN_LO2 = 1_000
FR_MAX_LO2 = 14_000

F_IF = 83160000  # Hz 1st IF frequency
F_RF_MIN = 2000000  # Hz min receive frequency
F_RF_MAX = 30020000  # Hz max receive frequency

R_MAX = 8191

def fast_find_magic_numbers(f, f_ref, fr_min, fr_max, allowed_error, maximize_fr = False):
    N_min = f / fr_max
    N_min_round = int(np.round(N_min) - 1)

    N_max = f / fr_min
    N_max_round = int(np.round(N_max) + 1)

    R_min = (N_min / f) * f_ref
    R_min_round = int(np.round(R_min) - 1)

    R_max = (N_max / f) * f_ref
    R_max_round = int(np.round(R_max) + 1)
    
    # limit R_max to the max MC145220 R division ratio of 8191
    if R_max_round > R_MAX:
        R_max_round = R_MAX
        
    N_range = range(N_min_round, N_max_round)

    if maximize_fr:
        R_range = range(R_min_round, R_max_round) # use for LO2  - start at a high F_r to minimize phase noise, also about 1kHz F_r does not seem to work?
    else:
        R_range = range(R_max_round, R_min_round, -1) # use for LO1 - start at a low F_r to prevent spurious locking
    
    f_r_range = [f_ref / item for item in R_range]

    f_actual = -1
    N_found = -1
    R_found = -1
    f_r_found = -1
    breakflag = False
    for f_r, R in zip(f_r_range, R_range):

        for N in N_range:

            if (abs(N * f_r - f) <= allowed_error):

                f_actual = N * f_r
                N_found = N
                R_found = R
                f_r_found = f_r
                breakflag = True
                break
        if breakflag:
            break

    return f_actual, N_found, R_found, f_r_found


def fast_lo1():

    # 85_160_000, 113_160_000, LO1_STEP)
    f_lo1_min = F_RF_MIN + F_IF
    f_lo1_max = F_RF_MAX + F_IF
    print(f"f_lo1_min: {f_lo1_min}")
    print(f"f_lo1_max: {f_lo1_max}")

    f_lo1_range = range(f_lo1_min, f_lo1_max, LO1_STEP)

    Ns = []
    Rs = []
    Frs = []
    f_actuals = []
    for f_lo1 in tqdm(f_lo1_range):
        f_actual, N, R, Fr = fast_find_magic_numbers(
            f_lo1, F_REF, FR_MIN_LO1, FR_MAX_LO1, ALLOWED_ERROR_LO1
        )
        f_actuals.append(f_actual)
        if f_actual != -1:

            Ns.append(N)
            Rs.append(R)
            Frs.append(Fr)
        else:
            Ns.append(-1)
            Rs.append(-1)
            Frs.append(-1)
            print(f"No solution found for {f_lo1}")

    errors = [f_actual - f_wanted for f_actual, f_wanted in zip(f_actuals, f_lo1_range)]

    df = pd.DataFrame(
        list(zip(Ns, Rs, Frs, f_lo1_range, f_actuals, errors)),
        columns=["N", "R", "f_r", "f_wanted", "f_actual", "error"],
    )

    df.to_csv("LO1_results.csv")


def fast_lo2():

    f_lo2_min = F_LO2_CENTER - int(LO1_STEP / 2) - ALLOWED_ERROR_LO1 - SIDEBANDMARGIN
    f_lo2_max = F_LO2_CENTER + int(LO1_STEP / 2) + ALLOWED_ERROR_LO1 + SIDEBANDMARGIN
    print(f"f_lo2_min: {f_lo2_min}")
    print(f"f_lo2_max: {f_lo2_max}")
    f_lo2_range = range(f_lo2_min, f_lo2_max, LO2_STEP)

    # 276 symmetry
    # 83612900

    Ns = []
    Rs = []
    Frs = []
    f_actuals = []

    for f_lo2 in tqdm(f_lo2_range):
        f_actual, N, R, Fr = fast_find_magic_numbers(
            f_lo2, F_REF, FR_MIN_LO2, FR_MAX_LO2, ALLOWED_ERROR_LO2, maximize_fr = True
        )
        f_actuals.append(f_actual)
        if f_actual != -1:

            Ns.append(N)
            Rs.append(R)
            Frs.append(Fr)
        else:
            Ns.append(-1)
            Rs.append(-1)
            Frs.append(-1)
            print(f"No solution found for {f_lo2}")

    errors = [f_actual - f_wanted for f_actual, f_wanted in zip(f_actuals, f_lo2_range)]

    df = pd.DataFrame(
        list(zip(Ns, Rs, Frs, f_lo2_range, f_actuals, errors)),
        columns=["N", "R", "f_r", "f_wanted", "f_actual", "error"],
    )

    df.to_csv("LO2_results.csv")



def fixgaps_lo1():
    
    f_lo1_min = F_RF_MIN + F_IF
    f_lo1_max = F_RF_MAX + F_IF

    f_lo1_range = range(f_lo1_min, f_lo1_max, LO1_STEP)
    
    # re-calculate points that cannot be made within the allowed F_r range
    gaps = [541, 543]

    f_lo1_range = range(f_lo1_min, f_lo1_max, LO1_STEP)
    gap_range = [f_lo1_range[item] for item in gaps]
        
    Ns = []
    Rs = []
    Frs = []
    f_actuals = []

    for f_lo1 in tqdm(gap_range):
        f_actual, N, R, Fr = fast_find_magic_numbers(
            f_lo1, F_REF, FR_MIN_LO1_GAPS, FR_MAX_LO1, ALLOWED_ERROR_LO1_GAPS, maximize_fr = True
        )
        f_actuals.append(f_actual)
        if f_actual != -1:

            Ns.append(N)
            Rs.append(R)
            Frs.append(Fr)
        else:
            Ns.append(-1)
            Rs.append(-1)
            Frs.append(-1)
            print(f"No solution found for {f_lo1}")

    errors = [f_actual - f_wanted for f_actual, f_wanted in zip(f_actuals, gap_range)]

    df = pd.DataFrame(
        list(zip(Ns, Rs, Frs, gap_range, f_actuals, errors)),
        columns=["N", "R", "f_r", "f_wanted", "f_actual", "error"],
    )

    df.to_csv("LO1_gap_results.csv")

def fixgaps_lo2():
    
    # re-calculate points that cannot be made within the possible MC145220 R divider range of 1 or 10 to 8191
    gaps = [489, 490, 491, 493, 494, 495, 975, 977]

    f_lo2_min = F_LO2_CENTER - int(LO1_STEP / 2) - ALLOWED_ERROR_LO1 - SIDEBANDMARGIN
    f_lo2_max = F_LO2_CENTER + int(LO1_STEP / 2) + ALLOWED_ERROR_LO1 + SIDEBANDMARGIN

    f_lo2_range = range(f_lo2_min, f_lo2_max, LO2_STEP)
    gap_range = [f_lo2_range[item] for item in gaps]
        
    Ns = []
    Rs = []
    Frs = []
    f_actuals = []

    for f_lo2 in tqdm(gap_range):
        f_actual, N, R, Fr = fast_find_magic_numbers(
            f_lo2, F_REF, FR_MIN_LO2, FR_MAX_LO2, ALLOWED_ERROR_LO2_GAPS, maximize_fr = True
        )
        f_actuals.append(f_actual)
        if f_actual != -1:

            Ns.append(N)
            Rs.append(R)
            Frs.append(Fr)
        else:
            Ns.append(-1)
            Rs.append(-1)
            Frs.append(-1)
            print(f"No solution found for {f_lo2}")

    errors = [f_actual - f_wanted for f_actual, f_wanted in zip(f_actuals, gap_range)]

    df = pd.DataFrame(
        list(zip(Ns, Rs, Frs, gap_range, f_actuals, errors)),
        columns=["N", "R", "f_r", "f_wanted", "f_actual", "error"],
    )

    df.to_csv("LO2_gap_results.csv")



#fast_lo1()
#fast_lo2()
#fixgaps_lo2()
fixgaps_lo1()