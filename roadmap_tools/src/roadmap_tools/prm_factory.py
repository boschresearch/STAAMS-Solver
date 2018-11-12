#!/usr/bin/env python
"""
Copyright (c) 2018 Robert Bosch GmbH
All rights reserved.

This source code is licensed under the BSD-3-Clause license found in the
LICENSE file in the root directory of this source tree.

@author: Jan Behrens
"""

from file_handler import FileHandler
from roadmap_tools.prm import RoadMap

import os
import lmdb

try:
    import cPickle as pickle
except ImportError:
    import pickle


class RoadMapFactory:
    def __init__(self):
        pass

    @staticmethod
    def export_from_db_to_file(fingerprint):
        prm = RoadMapFactory.load_prm_from_database(fingerprint)
        assert isinstance(prm, RoadMap)
        return RoadMapFactory.save_to_file(prm)

    @staticmethod
    def save_to_file(prm_save):
        assert isinstance(prm_save, RoadMap)
        with open(prm_save.get_fingerprint() + '.pkl', 'wb') as output:
            pickle.dump(prm_save, output, pickle.HIGHEST_PROTOCOL)

        # find absolute file path
        FOLDER_PATH = os.path.dirname(os.path.abspath(__file__))
        FILE_PATH = os.path.join(FOLDER_PATH, prm_save.get_fingerprint() + ".pkl")
        assert os.path.isfile(FILE_PATH)
        # rospy.loginfo("Saved Roadmap {}".format(file_name))
        return FILE_PATH


    @staticmethod
    def load_from_file(file_name):
        f = FileHandler()
        loaded_prm = f.load_prm(file_name)
        assert isinstance(loaded_prm, RoadMap)
        # format_version = loaded_prm["format_version"]
        return loaded_prm

    @staticmethod
    def is_db_present():
        DB_PATH = os.path.expanduser("~/lmdb/roadmaps")
        INDEX_PATH = os.path.expanduser("~/lmdb/index")
        return os.path.isfile(DB_PATH + "/data.mdb") and os.path.isfile(INDEX_PATH + "/data.mdb")

    @staticmethod
    def save_prm_to_database(prm, replace=True, suffix="snapshot"):
        # type: (RoadMap) -> bool
        DB_PATH = os.path.expanduser("~/lmdb/roadmaps")

        key = RoadMapFactory.get_key_for_database(prm.get_fingerprint(), db="roadmap")
        if not replace:
            prm.generate_fingerprint(prm.get_fingerprint() + suffix)
            key = RoadMapFactory.get_key_for_database(prm.get_fingerprint(), db="roadmap")
        assert key is not None

        print('----- Saving data -----')
        map_size = 1 * int(1e12)
        db_handle = lmdb.open(DB_PATH, map_size=map_size)
        with db_handle.begin(write=True) as txn:
            if txn.get("{:08d}".format(key)) is None:
                im_dat = pickle.dumps(prm)
                txn.put("{:08d}".format(key), im_dat)
                print("Saving RoadMap with fingerprint {} to {}.".format(prm.get_fingerprint(), DB_PATH))
            else:
                im_dat = pickle.dumps(prm)
                txn.replace("{:08d}".format(key), im_dat)
                print("Replacing RoadMap with fingerprint {} to {}.".format(prm.get_fingerprint(), DB_PATH))

        RoadMapFactory.add_key_to_dictionary(prm.get_fingerprint(), key, db="roadmap")
        return True

    @staticmethod
    def add_key_to_dictionary(fingerprint, key, db="roadmap"):

        DB_PATH = os.path.expanduser("~/lmdb/index")
        dict_key = None
        if db == "roadmap":
            dict_key = "001"  # key of dictionary with roadmap keys
        if db == "clash":
            dict_key = "002"  # key of dictionary with roadmap keys
        if db == "solutions":
            dict_key = "003"  # key of dictionary with roadmap keys

        if dict_key is None:
            print "I have no dictionary {}".format(db)
            return None

        print('----- Saving data -----')
        map_size = 1 * int(1e12)
        db_handle = lmdb.open(DB_PATH, map_size=map_size)
        with db_handle.begin(write=True) as txn:
            raw_dict = txn.get(dict_key)
            if raw_dict is not None:
                index = pickle.loads(raw_dict)  # type: dict[str, str]
            else:
                index = {}
                print "Created new index for {}.".format(db)

            index[fingerprint] = key
            raw_data = pickle.dumps(index, pickle.HIGHEST_PROTOCOL)
            txn.replace(dict_key, raw_data)

        return True

    @staticmethod
    def delete_and_init_dbs():

        DB_PATH = os.path.expanduser("~/lmdb/index")

        print('----- Saving data -----')
        map_size = 1 * int(1e12)
        db_handle = lmdb.open(DB_PATH, map_size=map_size)
        with db_handle.begin(write=True) as txn:
            db = db_handle.open_db()
            txn.drop(db)

        DB_PATH = os.path.expanduser("~/lmdb/roadmaps")

        print('----- Saving data -----')
        map_size = 1 * int(1e12)
        db_handle = lmdb.open(DB_PATH, map_size=map_size)
        with db_handle.begin(write=True) as txn:
            db = db_handle.open_db()
            txn.drop(db)

        return True

    @staticmethod
    def get_key_for_database(fingerprint, db="roadmap"):
        """
        This Method checks if a key already exists for the given fingerprint. If yes, the key is returned. Otherwise, we find the highest key and return an incremented key.
        :param fingerprint:
        :param db: "roadmap", "clash" or "solutions"
        :return:
        """
        DB_PATH = os.path.expanduser("~/lmdb/index")
        dict_key = None
        if db == "roadmap":
            dict_key = "001"  # key of dictionary with roadmap keys
        if db == "clash":
            dict_key = "002"  # key of dictionary with roadmap keys
        if db == "solutions":
            dict_key = "003"  # key of dictionary with roadmap keys

        if dict_key is None:
            print "I have no dictionary {}".format(db)
            return None
        print('----- Loading data -----')
        db_handle = lmdb.open(DB_PATH, readonly=True)
        with db_handle.begin() as txn:
            if txn.get(dict_key) is None:
                return 0
            else:
                raw_dict = txn.get(dict_key)

        index = pickle.loads(raw_dict)  # type: dict[str, str]
        max_key = 0
        for fingerprint_db, key in index.items():
            if fingerprint_db == fingerprint:
                return index[fingerprint_db]
            if key > max_key:
                max_key = key

        new_key = max_key + 1
        return new_key

    @staticmethod
    def read_key_from_database(fingerprint, db="roadmap"):
        # type: (str) -> [str, None]
        DB_PATH = os.path.expanduser("~/lmdb/index")
        dict_key = None
        if db == "roadmap":
            dict_key = "001"  # key of dictionary with roadmap keys
        if db == "clash":
            dict_key = "002"  # key of dictionary with roadmap keys
        if db == "solutions":
            dict_key = "003"  # key of dictionary with roadmap keys
        if dict_key is None:
            print "I have no dictionary {}".format(db)
            return None

        print('----- Loading data -----')
        db_handle = lmdb.open(DB_PATH, readonly=True)
        with db_handle.begin() as txn:
            raw_dict = txn.get(dict_key)

        try:
            index = pickle.loads(raw_dict)
            key = index[fingerprint]
            return key
        except KeyError:
            print "There is no key for fingerprint {}".format(fingerprint)
        return None

    @staticmethod
    def load_prm_from_database(fingerprint):
        # type: (str) -> bool

        key = RoadMapFactory.read_key_from_database(fingerprint, db="roadmap")
        assert key is not None

        DB_PATH = os.path.expanduser("~/lmdb/roadmaps")
        print('----- Loading data -----')
        db_handle = lmdb.open(DB_PATH, readonly=True)
        with db_handle.begin() as txn:
            dat = txn.get("{:08d}".format(key))
            prm = pickle.loads(dat)
        return prm

    @staticmethod
    def print_prm_from_database():
        # type: () -> None

        # key = RoadMapFactory.read_key_from_database(fingerprint, db="roadmap")
        # assert key is not None

        DB_PATH = os.path.expanduser("~/lmdb/roadmaps")
        print('----- Loading data -----')
        db_handle = lmdb.open(DB_PATH, readonly=True)
        with db_handle.begin() as txn:
            cursor = txn.cursor()
            cursor.first()
            it = cursor.iternext(keys=True, values=True)
            for (idx, (key, value)) in enumerate(it):
                dat = bytes(value)
                item = pickle.loads(dat)  # type: RoadMap
                print("Roadmap {} with db_index {}".format(item.get_fingerprint(), idx))
        return None

    @staticmethod
    def get_prm_index():
        DB_PATH = os.path.expanduser("~/lmdb/index")
        db = "roadmap"
        dict_key = None
        if db == "roadmap":
            dict_key = "001"  # key of dictionary with roadmap keys
        if db == "clash":
            dict_key = "002"  # key of dictionary with roadmap keys
        if db == "solutions":
            dict_key = "003"  # key of dictionary with roadmap keys
        if dict_key is None:
            print "I have no dictionary {}".format(db)
            return None

        print('----- Loading data -----')
        db_handle = lmdb.open(DB_PATH, readonly=True)
        with db_handle.begin() as txn:
            raw_dict = txn.get(dict_key)
            if raw_dict is None:
                print "There is no entry in the db for {}".format(db)
                return

        try:
            index = pickle.loads(raw_dict)
            return index
        except KeyError:
            print "There is no key for fingerprint {}".format(fingerprint)
        return None
