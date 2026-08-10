# Copyright 2020 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

import re
from dataclasses import dataclass


@dataclass
class RiscvISA:
    """Contain a valid base ISA string"""

    i: bool = False
    e: bool = False
    m: bool = False
    a: bool = False
    f: bool = False
    d: bool = False

    isa_string = re.compile(r"^rv32(i|e)([m|a|f|d]*)$")


def parse_isa_string(s):
    """Construct an `RiscvISA` object from a string"""
    s.lower()
    isa = RiscvISA()
    m = RiscvISA.isa_string.match(s)
    if m:
        setattr(isa, m.group(1), True)
        if m.group(2):
            [setattr(isa, t, True) for t in m.group(2)]
    else:
        raise ValueError("Illegal ISA string.")

    return isa
