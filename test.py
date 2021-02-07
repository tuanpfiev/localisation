#! /usr/bin/python3

from localisation_balloons import balloon_main
from navpy import lla2ned
import socket, sys
from _thread import *
import threading
import numpy as np
import csv
from datetime import datetime
import time
import os
import re
import math


if __name__ == "__main__":
    lon_ref = 142.1962
    lat_ref = -36.7189
    alt_ref = 0

    ned = lla2ned(lat_ref, lon_ref, 100, lat_ref, lon_ref, alt_ref)
    print(ned)
    