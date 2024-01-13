cmake_minimum_required(VERSION 3.3)

project(MD5Mesh2GLTF)

build_target(
  NAME
  md5mesh_to_gltf
  TYPE
  ESMODULE
  VERSION
  1
  SOURCES
  md5mesh_to_gltf.cpp
  LINKS
  spike-interface
  gltf-interface
  AUTHOR
  "Lukas Cone"
  DESCR
  "Convert MD5 Mesh to GLTF"
  START_YEAR
  2023)
