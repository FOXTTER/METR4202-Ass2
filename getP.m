function [ new_px, new_py ] = getP( px, py, cx, cy )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    new_px = 3*px + 284 - cx;
    new_py = 3*py - cy;

end

