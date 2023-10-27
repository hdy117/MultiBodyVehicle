@echo off

SET protoc=%~dp0\..\deps\protoc.exe
SET grpc_cpp_plugin=%~dp0\..\deps\grpc_cpp_plugin.exe
SET msg_dir=%~dp0

for %%G in (%msg_dir%\*.proto) do %protoc% --proto_path=%msg_dir%\..\deps -I=%msg_dir% --cpp_out=%msg_dir% %%G