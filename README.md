# Airspeed Kalman Filter
2023
By Frédéric Larocque as part of master thesis at TU Delft in the Control and Simulation Section

Link to thesis: https://repository.tudelft.nl/islandora/object/uuid%3A5d786e19-6871-4478-bda8-43f7cab20633

# Description
This work proposes a novel synthetic air data system for the Variable Skew Quad Plane (VSQP) hybrid vehicle to allow airspeed estimation from hover to high speed forward flight and provide pitot tube fault detection. An Extended Kalman Filter fuses Global Navigation Satellite System (GNSS) and inertial measurements using model-independent kinematics equations to estimate wind and airspeed without the use of the pitot tube. The filter is augmented by a simplified vehicle force model. Pitot tube fault detection is achieved with a simple thresholding operation on the pitot tube measurement and the airspeed estimation residual. Accurate airspeed estimation was validated with logged test flight data, achieving an overall 1.62 m/s root mean square error. Using the airspeed estimation, quick detection (0.16 s) of a real-life abrupt pitot tube fault was demonstrated. This new airspeed estimation method provides an innovative approach for increasing the fault tolerance of the VSQP and similar quad-plane vehicles.

# What is on this repo?
This repo only contains the matlab scripts used to design the kalman filter at the heart of the synthetic air data system. To access C Code generated for VSQP: https://github.com/frlarocque/rotWingPaparazzi/tree/airspeed_wind_EKF

# Table of Contents

## Figures
Figures generated for thesis + progress reports

## Force_based
Scripts to process the wind tunnel data and generate the force models for the different components (wing, body, hover propellers, pusher propellers) of VSQP. Wind tunnel data not included as file size too big for github.

## Functions
Various functions used to generate figures, run the kalman filter, process data, etc.

## Kalman_filter_based
Main folder where the synthetic air data system is found. "single_run_complete_filter.m" is the main script to run to estimate airspeed using other sensors data.

## Mat_files
Empty

## Varia
Other scripts used in thesis, but of limited value.
