#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

import cPickle as pickle


class file_handler:
    def load_prm(self, file_name="prm"):
        with open(file_name + '.pkl', 'rb') as in_file:
            loaded_prm = pickle.load(in_file)
        return loaded_prm

    def save_clash(self, left, right, file_name="prm_clash"):
        prm_clash = {"left_arm": left, "right_arm": right}
        with open(file_name + '.pkl', 'wb') as output:
            pickle.dump(prm_clash, output, pickle.HIGHEST_PROTOCOL)

    def load_clash(self, file_name="prm_clash"):
        with open(file_name + '.pkl', 'rb') as output:
            loaded_clash = pickle.load(output)

        return loaded_clash


class FileHandler:
    def __init__(self):
        pass

    def load_prm(self, file_name="prm"):
        if not ".pkl" in file_name:
            file_name += ".pkl"
        with open(file_name, 'rb') as in_file:
            loaded_prm = pickle.load(in_file)
        return loaded_prm

    def save_clash(self, left, right, file_name="prm_clash"):
        prm_clash = {"left_arm": left, "right_arm": right}
        with open(file_name + '.pkl', 'wb') as output:
            pickle.dump(prm_clash, output, pickle.HIGHEST_PROTOCOL)

    def load_clash(self, file_name="prm_clash"):
        with open(file_name + '.pkl', 'rb') as output:
            loaded_clash = pickle.load(output)

        return loaded_clash
