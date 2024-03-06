@echo off

SET protoc=E:\work\vcpkg\installed\x64-windows\tools\protobuf\protoc.exe
SET grpc_cpp_plugin=E:\work\vcpkg\installed\x64-windows\tools\grpc\grpc_cpp_plugin.exe
SET msg_dir=%~dp0

for %%G in (%msg_dir%\*.proto) do %protoc%  -I=%msg_dir% --cpp_out=%msg_dir% %%G