// libfpv - OS specifics.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "util/platform.h"

#include <atomic>
#include <cstdlib>
#include <cstring>

#include "util/log.h"

#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#  include <shellapi.h>
#  include <shlobj.h>
#elif !defined(__EMSCRIPTEN__)
#  include <dlfcn.h>
#endif

namespace fpv {

#if defined(_WIN32)
static std::string utf8(const wchar_t *w) {
    int bytes = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
    if (bytes <= 1) return {};
    std::string out(size_t(bytes - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, w, -1, out.data(), bytes, nullptr, nullptr);
    return out;
}
#endif

std::string envUtf8(const char *name) {
#if defined(_WIN32)
    // The process environment, not the C runtime's copy: MinGW's msvcrt only
    // builds its wide table for wmain programs, so _wgetenv returns null here.
    std::wstring wname(name, name + std::strlen(name));
    DWORD n = GetEnvironmentVariableW(wname.c_str(), nullptr, 0);
    if (n == 0) return {};
    std::wstring value(n, L'\0');
    GetEnvironmentVariableW(wname.c_str(), value.data(), n);
    return utf8(value.c_str());
#else
    const char *v = std::getenv(name);
    return v ? v : "";
#endif
}

std::string userConfigDir() {
#if defined(_WIN32)
    // The shell's answer, not %APPDATA%: some processes start without it.
    PWSTR path = nullptr;
    std::string dir;
    if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_RoamingAppData, 0, nullptr, &path))) dir = utf8(path);
    CoTaskMemFree(path);
    return dir.empty() ? envUtf8("APPDATA") : dir;
#elif defined(__APPLE__)
    std::string home = envUtf8("HOME");
    return home.empty() ? home : home + "/Library/Application Support";
#else
    std::string xdg = envUtf8("XDG_CONFIG_HOME"), home = envUtf8("HOME");
    return !xdg.empty() ? xdg : home.empty() ? home : home + "/.config";
#endif
}

bool openFolder(const std::string &utf8Dir) {
#if defined(_WIN32)
    int n = MultiByteToWideChar(CP_UTF8, 0, utf8Dir.c_str(), -1, nullptr, 0);
    if (n <= 0) return false;
    std::wstring w(n, L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8Dir.c_str(), -1, &w[0], n);
    return (INT_PTR)ShellExecuteW(nullptr, L"open", w.c_str(), nullptr, nullptr, SW_SHOWNORMAL) > 32;
#else
    (void)utf8Dir;
    return false;
#endif
}

static std::atomic<bool> g_pinned{false};

bool pinLibraryInMemory() {
    if (g_pinned) return true;
#if defined(__EMSCRIPTEN__)
    // linked statically into the page's module: nothing can unload it
#elif defined(_WIN32)
    HMODULE h = nullptr;
    if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN,
                            (LPCWSTR)(void *)&pinLibraryInMemory, &h)) {
        FPV_LOGW("could not pin libfpv in memory (error %lu)", (unsigned long)GetLastError());
        return false;
    }
#else
    Dl_info info;
    if (!dladdr((void *)&pinLibraryInMemory, &info) || !info.dli_fname) {
        FPV_LOGW("could not locate libfpv to pin it in memory");
        return false;
    }
    // An extra reference flagged NODELETE: dlclose() never unmaps us.
    void *h = dlopen(info.dli_fname, RTLD_NOW | RTLD_NOLOAD | RTLD_NODELETE);
    if (!h) {
        FPV_LOGW("could not pin libfpv in memory: %s", dlerror());
        return false;
    }
#endif
    g_pinned = true;
    return true;
}

bool libraryIsPinned() { return g_pinned; }

static void (*g_exitFn)() = nullptr;

void installExitHandler(void (*fn)()) {
#if defined(_WIN32)
    (void)fn;
#else
    g_exitFn = fn;
    std::atexit([] {
        if (g_exitFn) g_exitFn();
    });
#endif
}

}  // namespace fpv
