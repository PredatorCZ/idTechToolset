cmake_minimum_required(VERSION 3.3)

project(MD5Anim2GLTF)

build_target(
  NAME
  md5anim_to_gltf
  TYPE
  ESMODULE
  VERSION
  1
  SOURCES
  md5anim_to_gltf.cpp
  LINKS
  spike-interface
  gltf-interface
  AUTHOR
  "Lukas Cone"
  DESCR
  "Convert MD5 Anim to GLTF"
  START_YEAR
  2023)
