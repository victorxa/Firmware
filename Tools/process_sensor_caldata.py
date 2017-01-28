#! /usr/bin/env python
"""
Reads in IMU data from a static thermal calibration test and performs
a curve fit of gyro, accel and baro bias vs temperature
Data can be gathered using the following sequence:

1) Set the TC_A_ENABLE, TC_B_ENABLE and TC_G_ENABLE parameters to 0 to
    thermal compensation and reboot
2) Perform a gyro and accel cal
2) Set the SYS_LOGGER parameter to 1 to use the new system logger
3) Set the SDLOG_MODE parameter to 3 to enable logging of sensor data
    for calibration and power off
4) Cold soak the board for 30 minutes
5) Move to a warm dry environment.
6) Apply power for 45 minutes, keeping the board still.
7) Remove power and extract the .ulog file
8) Open a terminal window in the script file directory
9) Run the script file 'python process_sensor_caldata.py
    <full path name to .ulog file>

Outputs thermal compensation parameters in a file named
    <inputfilename>.params which can be loaded onto the
    board using QGroundControl
Outputs summary plots in a pdf file named <inputfilename>.pdf

"""

from __future__ import print_function

import argparse
import json
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

import pandas
from numpy.polynomial import Polynomial
import numpy as np
import pyulog

# pylint: disable=invalid-name


def ulog2pandas(log):
    """
    Convert from ulog to pandas using dictionary
    """
    r = {}
    for msg in log.data_list:
        data = pandas.DataFrame.from_dict(msg.data, dtype=float)
        data.index = pandas.TimedeltaIndex(data.timestamp, 'us')
        if msg.name not in r.keys():
            r[msg.name] = {}
        r[msg.name][msg.multi_id] = data
    return r


def fitPlot(x, y, f_poly, name, field, config):
    """
    A temperature calibration fit plot.
    """
    # pylint: disable=too-many-arguments
    x = x.resample(config['plot_interval']).mean()
    y = y.resample(config['plot_interval']).mean()
    plt.plot(x, y, '.', label=field)
    x_resample = np.linspace(x.min(), x.max())
    plt.plot(x_resample, f_poly(x_resample), '-',  label='{:s} fit'.format(field))
    plt.title('{:s} temperature calibration'.format(name))
    plt.xlabel(config['xlabel'])
    plt.ylabel(config['ylabel'])
    plt.legend(loc='best', ncol=3)

def temperature_calibration(ulog_filename, do_plot):
    """
    Do a temperature calibration
    @param ulog_filename : log filename
    @do_plot : create plots, save to pdf
    """
    log = pyulog.ULog(ulog_filename)
    r = ulog2pandas(log)
    coeffs = {}

    pdf_filename = "{:s}_temp_cal.pdf".format(
        ulog_filename.replace('.ulg',''))

    # default for config
    config = {
        'fields': [],
        'units': [],
        'poly_deg': 3,
        'xlabel': 'Temp, deg C',
        'ylabel': '',
        'min': lambda x: x.min(),
        'max': lambda x: x.max(),
        'ref': lambda x: (x.min() + x.max())/2,
        'offset': lambda y: 0,
        'save_plot': True,
        'plot_interval': '5 s',
    }

    pdf = PdfPages(pdf_filename)

    for topic in r.keys():
        for multi_id in r[topic].keys():

            name = '{:s}_{:d}'.format(topic, multi_id)
            print('processing', name)

            if topic == 'sensor_baro':
                config['fields'] = ['altitude']
                config['ylabel'] = 'pressure altitude, m'
                config['offset'] = lambda y: y.median()
            elif topic == 'sensor_gyro':
                config['fields'] = ['x', 'y', 'z']
                config['ylabel'] = 'gyro, rad/s'
            elif topic == 'sensor_accel':
                config['fields'] = ['x', 'y', 'z']
                config['ylabel'] = 'accel, m/s^2'
                config['offset'] = lambda y: y.median()
            else:
                continue

            # resample at 10 second interval using mean of sample
            # won't significantly impact coefficients,
            # but will save time and make plots smaller more readable
            data = r[topic][multi_id].ffill().bfill()

            x = data.temperature

            # default for coefficients
            coeffs[name] = {
                'poly': {},
                'T_min': config['min'](x),
                'T_max': config['max'](x),
                'T_ref': config['ref'](x),
            }

            plt.figure()

            for i, field in enumerate(config['fields']):
                y = data[field]
                y -= config['offset'](y)
                f_poly = Polynomial.fit(x, y, config['poly_deg'])
                coeffs[name]['poly'][field] = list(f_poly.coef)

                if do_plot:
                    fitPlot(
                        x=x, y=y, f_poly=f_poly, name=name, field=field,
                        config=config)
                    if config['save_plot'] and i == len(config['fields']) - 1:
                        pdf.savefig()
                        plt.close()

    pdf.close()

    with open(ulog_filename.replace('ulg', 'params'), 'w') as fid:
        fid.write(json.dumps(coeffs, indent=2, sort_keys=True))

if __name__ == "__main__":
    parser = argparse.ArgumentParser('Temperature calibration')
    parser.add_argument('ulog')
    parser.add_argument('--noplot', action='store_true', default=False)
    args = parser.parse_args()
    temperature_calibration(args.ulog, not args.noplot)

#  vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 :
