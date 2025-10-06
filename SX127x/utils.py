""" Some utility functions. """
# -*- coding: utf-8 -*-

# Copyright 2015-2025 Mayer Analytics Ltd.
#
# This file is part of pySX127x.
#
# pySX127x is free software: you can redistribute it and/or modify it under the terms of the GNU Affero General Public
# License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# pySX127x is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Affero General Public License for more
# details.
#
# You can be released from the requirements of the license by obtaining a commercial license. Such a license is
# mandatory as soon as you develop commercial activities involving pySX127x without disclosing the source code of your
# own applications, or shipping pySX127x with a closed source product.
#
# You should have received a copy of the GNU General Public License along with pySX127.  If not, see
# <http://www.gnu.org/licenses/>.

import sys

def set_bit(value, index, new_bit):
    """ Set the index-th bit of value to new_bit, and return the new value.
    :param value:   The integer to set the new_bit in
    :type value:    int
    :param index: 0-based index
    :param new_bit: New value the bit shall have (0 or 1)
    :return:    Changed value
    :rtype: int
    """
    mask = 1 << index
    value &= ~mask
    if new_bit:
        value |= mask
    return value


def getter(register_address):
    """ The getter decorator reads the register content and calls the decorated function to do
        post-processing.
    :param register_address: Register address
    :return: Register value
    :rtype: int
    """
    def decorator(func):
        def wrapper(self):
            return func(self, self.spi.xfer([register_address, 0])[1])
        return wrapper
    return decorator


def setter(register_address):
    """ The setter decorator calls the decorated function for pre-processing and
        then writes the result to the register
    :param register_address: Register address
    :return: New register value
    :rtype: int
    """
    def decorator(func):
        def wrapper(self, val):
            return self.spi.xfer([register_address | 0x80, func(self, val)])[1]
        return wrapper
    return decorator
