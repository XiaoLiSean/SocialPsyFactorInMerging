clear all; clc
close all;

addpath('./HighDExperiment');
scene_num   = '60';
which_side  = 'upper';
[startFrame, endFrame]  = deal(6852, 7352);
[params, vizHistory]    = preprocessingHighD(scene_num, which_side);
generateHighDAnimation(params, vizHistory, scene_num, which_side, startFrame, endFrame);