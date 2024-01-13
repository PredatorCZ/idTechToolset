cmake_minimum_required(VERSION 3.3)

project(DEFDump)

build_target(
  NAME
  def_dump
  TYPE
  ESMODULE
  VERSION
  1
  SOURCES
  def_dump.cpp
  LINKS
  spike-interface
  AUTHOR
  "Lukas Cone"
  DESCR
  "Dump def events into QC"
  START_YEAR
  2023)
