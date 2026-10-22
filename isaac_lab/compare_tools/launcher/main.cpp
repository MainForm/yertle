#include <windows.h>
#include <shellapi.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "config.hpp"

std::wstring quote(const std::wstring& s){
    std::wstring out=L"\"";std::size_t slashes=0;
    for(auto c:s){
        if(c==L'\\'){++slashes;continue;}
        if(c==L'\"')out.append(slashes*2+1,L'\\');else out.append(slashes,L'\\');
        slashes=0;out+=c;
    }
    out.append(slashes*2,L'\\');out+=L'\"';return out;
}
std::wstring command(const std::vector<std::wstring>& args){
    std::wstring out;for(const auto& s:args){if(!out.empty())out+=L' ';out+=quote(s);}return out;
}
int selfTest(){
    const std::vector<std::wstring> expected={L"test.exe",L"",L"plain",L"space path",L"C:\\한글 폴더\\",L"a\\\"b",L"--log"};
    const auto line=command(expected);int count{};auto parsed=CommandLineToArgvW(line.c_str(),&count);
    if(!parsed||count!=static_cast<int>(expected.size()))throw std::runtime_error("Argument parsing failed");
    for(int i=0;i<count;++i)if(parsed[i]!=expected[i]){LocalFree(parsed);throw std::runtime_error("Argument mismatch");}
    LocalFree(parsed);std::cout<<"PASS Unicode/space/quote/trailing-slash argument roundtrip; no process or COM opened\n";return 0;
}
int wmain(int argc,wchar_t** argv){try{
    if(argc==2&&std::wstring(argv[1])==L"--self-test")return selfTest();
    if(argc==2&&std::wstring(argv[1])==L"--help"){
        std::wcout<<L"Yertle read-only real / independent SIM viewer\n"
            <<L"--python PATH optionally selects an existing Isaac Python.\n"
            <<L"Viewer options: --demo | --log FILE | --replay FILE; --model FILE; --seconds N; --capture\n"
            <<L"Default follows the current CLI robot_*.jsonl; it never opens COM or commands hardware.\n"
            <<L"Script: "<<kViewerScript<<L'\n';return 0;
    }
    std::wstring python=kIsaacPython;std::vector<std::wstring> extra;bool experience=false;
    for(int i=1;i<argc;++i){
        std::wstring a=argv[i];if(a==L"--python"){if(++i==argc)throw std::invalid_argument("Missing --python path");python=argv[i];continue;}
        if(a==L"--experience"||a.rfind(L"--experience=",0)==0)experience=true;
        extra.push_back(a);
    }
    if(!std::filesystem::is_regular_file(python)||!std::filesystem::is_regular_file(kViewerScript))
        throw std::runtime_error("Missing Isaac Python/viewer script; rebuild here or supply --python PATH");
    std::vector<std::wstring> args={python,L"-u",kViewerScript};args.insert(args.end(),extra.begin(),extra.end());
    wchar_t temp[MAX_PATH],status[MAX_PATH];
    if(!GetTempPathW(MAX_PATH,temp)||!GetTempFileNameW(temp,L"ycv",0,status))throw std::runtime_error("Cannot allocate viewer status file");
    args.push_back(L"--exit-status-file");args.push_back(status);
    if(!experience){args.push_back(L"--experience");args.push_back(L"isaaclab.python.yertle_min_d3d12.kit");}
    auto line=command(args);STARTUPINFOW si{};si.cb=sizeof(si);PROCESS_INFORMATION pi{};
    if(!CreateProcessW(python.c_str(),line.data(),nullptr,nullptr,TRUE,CREATE_NO_WINDOW,nullptr,nullptr,&si,&pi)){
        std::cerr<<"CreateProcess failed, Windows error="<<GetLastError()<<'\n';return 1;
    }
    std::cout<<"Viewer launched (no COM). Close only its Isaac window to exit.\n";
    CloseHandle(pi.hThread);WaitForSingleObject(pi.hProcess,INFINITE);DWORD code{};
    if(!GetExitCodeProcess(pi.hProcess,&code)){CloseHandle(pi.hProcess);return 1;}
    CloseHandle(pi.hProcess);
    if(code)return static_cast<int>(code);
    std::ifstream status_file{std::filesystem::path(status)};char result='?';status_file>>result;
    if(result!='0'){std::cerr<<"Viewer failed; inspect its report/traceback (Kit may return process code 0 on shutdown).\n";return 1;}
    return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
