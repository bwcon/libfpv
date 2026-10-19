// libfpv - OS specifics.
// SPDX-License-Identifier: GPL-3.0-or-later
#include "util/platform.h"

#include <atomic>
#include <cstdlib>

#include "util/log.h"

#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  include <windows.h>
#elif !defined(__EMSCRIPTEN__)
#  include <dlfcn.h>
#endif

namespace fpv {

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
