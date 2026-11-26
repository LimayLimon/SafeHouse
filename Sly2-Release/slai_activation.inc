// Include once, inside activate.cpp's anonymous namespace, after FAlo.
// The center is provided by the Slai guest module; the native camera/player remain untouched.
namespace slai_extra {
bool Enabled() {
    static const bool on = [] { const char* p = std::getenv("SLY2N_SLAIACT"); return p && p[0]=='1' && p[1]=='\0'; }();
    return on;
}
bool Eligible(SW* psw, LO* root) {
    if (!root || !root->fInWorld || root->fRemoved || root->paloParent || root->fSpareNs) return false;
    if (root->Cid()==0x30 || root->Cid()==0x5B) return true; // basic ALO and coins
    if (FModSlaiOrdinaryGuard(root)) return true;
    // Only permit a streaming proxy that already owns an instance of an ordinary guard.
    // Do not request additional pool rows in this first conservative experiment.
    auto it = psw->mpproxy.find(root);
    if (it == psw->mpproxy.end() || it->second.airow.empty()) return false;
    const int i = it->second.iwe;
    if (i < 0 || size_t(i) >= psw->awe.size()) return false;
    const auto& w = psw->awe[size_t(i)];
    if (w.cloLoaded <= 0 || w.aplo.empty()) return false;
    const int row = it->second.airow.front();
    if (row < 0 || size_t(row) >= w.aploRowOwner.size() || w.aploRowOwner[size_t(row)] != root) return false;
    // A proxy group may have multiple roots. Restrict it to a recognised ordinary guard as its first root.
    return FModSlaiOrdinaryGuard(w.aplo.front());
}
bool Near(SW* psw, LO* root) {
    if (!Enabled() || !root || (root->st != 0 && root->st != 3) || !(g_actview.grfCm & root->bMask)) return false;
    glm::vec3 center(0.f);
    if (!FModSlaiActivationCenter(psw, &center)) return false;
    if (!slai_activation::Within(glm::distance(PosLoWorld(root), center), root->sRadius, root->fNodeActive)) return false;
    return Eligible(psw, root);
}
void RefreshQueue(SW* psw) {
    if (!Enabled()) return;
    static uint32_t serial = 0;
    static bool had = false;
    static glm::vec3 prev(0.f);
    static float last = 0.f;
    glm::vec3 center(0.f);
    const bool has = FModSlaiActivationCenter(psw, &center);
    const bool changed = serial != psw->serial;
    if (slai_activation::Refresh(had, has, changed, g_clock.t-last, has && had && !changed ? glm::distance(center,prev) : 0.f)) {
        psw->ploStop = nullptr;
        last = g_clock.t;
        prev = center;
        if (const char* log = std::getenv("SLY2N_SLAIACTLOG"); log && log[0]=='1')
            std::printf("Slai activation: world %u active %d center %.0f %.0f %.0f\n", unsigned(psw->serial), int(has), center.x, center.y, center.z);
    }
    serial = psw->serial;
    had = has;
}
}
