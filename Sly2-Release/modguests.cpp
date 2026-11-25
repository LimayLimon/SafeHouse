// See modguests.h.
#include "modguests.h"

#include "activate.h"
#include "ai.h"
#include "aijobs.h"
#include "clock.h"
#include "crv.h"
#include "po.h"
#include "aseg.h"
#include "asega.h"
#include "sm.h"
#include "brx.h"
#include "cls.h"
#include "disc.h"
#include "guard.h"
#include "damage.h"
#include "bsp.h"
#include "pccharacter.h"
#include <random>
#include "lo.h"
#include "modworld.h"
#include "pcloader.h"
#include "so.h"
#include "step.h"
#include "sw.h"
#include "transition.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace
{
struct LIVE
{
    const cane::mods::Guest* pg = nullptr;
    LO* plo = nullptr;
    bool fHold = false;   // (a player class: kept at its place)
    bool fSlai = false;  // opt-in: independent Sly guest, never the player
    int hp = 400;
    LO* path = nullptr;
    ASEG* walk = nullptr;
    ASEG* idle = nullptr;
    ASEG* run = nullptr;
    ASEG* hurt = nullptr;
    ASEG* defeat = nullptr;
    ASEG* playing = nullptr;
    std::vector<ASEG*> idles, attacks;
    ASEG* restClip = nullptr;
    LO* attackTarget = nullptr;
    float restUntil = 0, nextRest = 0, nextSwitch = 0;
    float hurtUntil = 0, respawnAt = 0, nextHit = 0;
    float attackUntil = 0, strikeAt = 0, nextAttack = 0;
    bool struck = false, fleeing = false;
    bool hasSpawn = false;
    glm::vec3 spawn{0.0f};
    float spawnS = 0, spawnHeading = 0;
    LO* spawnPath = nullptr;
    float s = 0.0f;
    int dir = 1;
    float heading = 0.0f;
    float nextSearch = 0.0f;
    glm::vec3 pos{0.0f};
    bool fPosition = false;
};
std::vector<LIVE> s_alive;
uint32_t s_serial = 0;
std::mt19937 s_rng{0x534C4149u};
constexpr float c_sSlaiWalk = 250.0f;
constexpr float c_sSlaiTravel = 510.0f;
constexpr float c_sSlaiRun = 830.0f;
float RandomRange(float a, float b) { return std::uniform_real_distribution<float>(a,b)(s_rng); }
float ClipTime(ASEG* a, float fallback) { return a && a->asegd.tMax > 0.05f ? std::clamp(a->asegd.tMax,0.1f,5.0f) : fallback; }

std::string WorldNameOf(int iwrec)
{
    const auto& awrec = AwrecWorlds();
    return iwrec >= 0 && iwrec < int(awrec.size()) ? g_disc.SzWorldName(awrec[std::size_t(iwrec)].fk.isector) : std::string();
}

// the guests of a world ("*": the command line's, every world)
std::vector<const cane::mods::Guest*> GuestsOf(const std::string& world)
{
    std::vector<const cane::mods::Guest*> a;
    if (world.empty()) return a;
    for (const cane::mods::Guest& g : g_disc.Aguest())
        if (g.world == world || g.world == "*") a.push_back(&g);
    return a;
}

int IwrecFrom(const cane::mods::Guest& g)
{
    const int iwrec = IwrecOfWorldArg(g.from);
    return iwrec >= 0 && !FModWorldRecord(iwrec) ? iwrec : -1;   // (installed levels only: a kept world is read from the disc)
}

bool FPrint() { return std::getenv("SLY2N_GUESTPRINT") != nullptr; }

void PlaceAt(LO* plo, const glm::vec3& pos, float rad)
{
    if (!FCidDerivedFrom(plo->Cid(), 0x30)) return;
    ALO* palo = static_cast<ALO*>(plo);
    const glm::mat3 m(glm::vec3(std::cos(rad), std::sin(rad), 0.0f), glm::vec3(-std::sin(rad), std::cos(rad), 0.0f), glm::vec3(0.0f, 0.0f, 1.0f));
    if (STEP* pstep = PstepFromLo(plo))
    {
        // (as a teleport places a guard: actor.cpp TeleportLo, orig 0x2052E0)
        SetStepPos(pstep, pos - m * glm::vec3(pstep->vecBase));
        SetStepRotation(pstep, m);
        SetAloVelocityTo(palo, glm::vec3(0.0f));
        SetAloAngVelocityTo(palo, glm::vec3(0.0f));
        pstep->pxpFloor = nullptr;
        return;
    }
    SetAloRotation(palo, m);
    SetAloPos(palo, pos);
}

void Place(LO* plo, const cane::mods::Guest& g)
{
    PlaceAt(plo, glm::vec3(g.at[0], g.at[1], g.at[2]), glm::radians(g.heading));
}

bool IsSlai(const cane::mods::Guest& g, const LO* plo)
{
    return FCidDerivedFrom(plo->Cid(), 0x35) &&
        (g.id == "slai" || g.id.rfind("slai-", 0) == 0 || g.name == "Slai");
}

// Native patrol path curves; no cast from SLY to GUARD. The first test is
// kinematic, not autonomous obstacle avoidance or general navigation.
LO* FindPatrol(SW* psw, const glm::vec3& anchor, float& bestS, LO* exclude = nullptr, float maxDistance = std::numeric_limits<float>::max())
{
    LO* best = nullptr;
    float bestDist = std::numeric_limits<float>::max();
    auto consider = [&](LO* path) {
        if (!path || path == exclude) return;
        const CRV* c = PcrvFromLo(path);
        if (!c || c->as.size() < 2 || !(c->as.back() - c->as.front() > 80.0f)) return;
        glm::vec3 q(0.0f);
        float s = 0.0f;
        NearestCrvAll(c, anchor, &q, nullptr, nullptr, &s);
        const glm::vec3 d = q - anchor;
        const float dist = glm::dot(d, d);
        if (std::isfinite(dist) && std::isfinite(s) && dist < bestDist && dist <= maxDistance * maxDistance)
        { best = path; bestS = s; bestDist = dist; }
    };
    // StartSpawner creates a CMDLIST (oid 0x2CB) with op 6 for each path source.
    // Also consider authored patrols already attached to original guards.
    auto scan = [&](const CMDLIST* list) {
        if (!list) return;
        for (const auto& cmd : list->acmd)
            if (cmd && cmd->op == 6) consider(PloFindSwObjectByOid(psw, cmd->oid));
    };
    for (const auto& obj : psw->aploAll)
    {
        LO* plo = obj.get();
        if (!plo) continue;
        if (plo->Cid() == 0x71 && plo->oid == 0x2CB) scan(static_cast<CMDLIST*>(plo));
        if (FCidDerivedFrom(plo->Cid(), 0x38))
            if (GUARD* guard = PguardFromLo(plo)) scan(guard->pcmdlistPatrol);
    }
    return best;
}

void PlayClip(LIVE& l, ASEG* want, bool loop = true, float rate = 1.0f)
{
    if (!want) want = l.idle;
    if (!want) return;
    ALO* palo = static_cast<ALO*>(l.plo);
    if (l.playing && l.playing != want)
        if (ASEGA* a = PasegaFindAseg(l.playing, palo)) RetractAsega(a);
    l.playing = want;
    if (ASEGA* a = PasegaFindAseg(want, palo))
    {
        if (std::abs(a->svtLocal - rate) > 0.001f) SetAsegaSpeed(a, rate);
        return;
    }
    if (ASEGA* a = PasegaApplyAseg(want, palo, 0.0f, rate, loop ? 9 : 33))
    {
        SetAsegaPriority(a, 20);
        SetAsegaSeekTime(a, 0.12f);
    }
}
void PlaySlai(LIVE& l, bool moving) { PlayClip(l, moving ? l.walk : l.idle); }
void RestartClip(LIVE& l, ASEG* clip)
{
    if (clip) if (ASEGA* a = PasegaFindAseg(clip, static_cast<ALO*>(l.plo))) RetractAsega(a);
    PlayClip(l, clip, false);
}
void FaceSlai(LIVE& l, const glm::vec3& target, float dt)
{
    const glm::vec3 d = target - l.pos;
    if (d.x*d.x+d.y*d.y < 1.0f) return;
    const float wanted = std::atan2(d.y,d.x);
    l.heading += std::clamp(std::remainder(wanted-l.heading, 6.2831853f), -6.0f*dt, 6.0f*dt);
}
float Distance(const glm::vec3& a,const glm::vec3& b) { return glm::length(a-b); }
// Native line test, using the same static-world filters as the guard detector.
bool SlaiClearLine(const glm::vec3& a, const glm::vec3& b)
{
    const glm::vec3 mn=glm::min(a,b), mx=glm::max(a,b);
    std::vector<SOE*> blockers;
    for (OXA* ox : g_physw.apoxaActive)
    {
        if (!ox || (ox->grf&4) || mx.x<ox->posMin.x || ox->posMax.x<mn.x || mx.y<ox->posMin.y || ox->posMax.y<mn.y || mx.z<ox->posMin.z || ox->posMax.z<mn.z) continue;
        for (SOE* e : ox->apsoe)
        {
            if (!e || e->f48 || !e->palo) continue;
            bool actor=false;
            for (const LO* par=e->palo; par; par=par->paloParent) if (FCidDerivedFrom(par->Cid(),0x33)) { actor=true; break; }
            if (!actor && blockers.size()<256) blockers.push_back(e);
        }
    }
    LSG hit;
    return blockers.empty() || !PsoeHitTestLineObjects(0,a,b,blockers,&hit);
}
bool LiveGuard(LO* o)
{
    if (!o || !o->fInWorld || o->fRemoved || !FCidDerivedFrom(o->Cid(),0x38)) return false;
    // Only ordinary guards, not boss/special subclasses. Positive native HP required.
    GUARD* g = PguardFromLo(o);
    return g && (o->Cid() == 0x38 || o->Cid() == 0x39) && g->n33C > 0 && !FGuardState(g,42);
}
LO* NearbyGuard(const glm::vec3& pos)
{
    LO* best = nullptr; float dist = 190.0f;
    for (GUARD* g : ApguardActive()) if (g && LiveGuard(g->palo))
    {
        const glm::vec3 q = PosAloCenter(g->palo);
        float d = Distance(pos + glm::vec3(0,0,65),q);
        if (d < dist && SlaiClearLine(pos+glm::vec3(0,0,65),q)) { best=g->palo; dist=d; }
    }
    return best;
}
void StrikeSlai(LIVE& l)
{
    LO* target = l.attackTarget;
    if (!LiveGuard(target)) return;
    const glm::vec3 q = PosAloCenter(static_cast<ALO*>(target));
    if (Distance(l.pos+glm::vec3(0,0,65),q)>195.0f || !SlaiClearLine(l.pos+glm::vec3(0,0,65),q)) return;
    CTMSG m; m.ploSender=l.plo; m.ploReceiver=target; m.grfFilter=4; m.nState=-20;
    m.pos=q; m.vec=glm::vec3(std::cos(l.heading),std::sin(l.heading),0.0f)*150.0f;
    PaloAcceptContact(&m);
}

bool StartSlaiPath(SW* psw, LIVE& l)
{
    if (!l.walk) return false;
    const bool automatic = l.pg->patrol < 0;
    LO* player = PloPoCurrent(psw);
    if (automatic && (!player || player == l.plo || !FCidDerivedFrom(player->Cid(), 0x30))) return false;
    glm::vec3 anchor = automatic ? PosAloCenter(static_cast<ALO*>(player)) :
        glm::vec3(l.pg->at[0], l.pg->at[1], l.pg->at[2]);
    float s = 0.0f;
    LO* path = automatic ? FindPatrol(psw, anchor, s) : PloFindSwObjectByOid(psw, l.pg->patrol);
    const CRV* c = PcrvFromLo(path);
    if (!c || c->as.size() < 2 || !(c->as.back() - c->as.front() > 80.0f)) return false;
    if (!automatic) NearestCrvAll(c, anchor, nullptr, nullptr, nullptr, &s);
    if (!std::isfinite(s)) return false;
    l.path = path;
    l.s = std::clamp(s, c->as.front(), c->as.back());
    l.dir = l.s >= c->as.back() - 1.0f ? -1 : 1;
    // Avoid spawning directly on the player on the automatic path.
    if (automatic)
    {
        float trial = l.s + float(l.dir) * 300.0f;
        if (trial > c->as.back() || trial < c->as.front()) trial = l.s - float(l.dir) * 300.0f;
        l.s = std::clamp(trial, c->as.front(), c->as.back());
    }
    std::printf("Slai: path %d, start %.1f, length %.1f, walk %d\n", path->oid, l.s, c->as.back() - c->as.front(), l.walk->oid);
    return true;
}

void StepSlai(SW* psw, LIVE& l)
{
    if (!l.plo->fInWorld || l.plo->fRemoved || l.plo == PloPoCurrent(psw)) return;
    const float now = g_clock.t, dt = std::clamp(g_clock.dt,0.0f,0.1f);
    if (FGamePausedForAi() || dt <= 0.0f) return;
    auto hold = [&] { if (l.fPosition) PlaceAt(l.plo,l.pos,l.heading); };
    if (l.hp <= 0)
    {
        if (l.respawnAt == 0) { l.respawnAt=now+6; RestartClip(l,l.defeat); }
        if (now < l.respawnAt) { hold(); return; }
        l.hp=400; l.respawnAt=0; l.hurtUntil=0; l.nextHit=now+2;
        l.attackUntil=0; l.attackTarget=nullptr; l.fleeing=false; l.restUntil=0;
        l.path=l.spawnPath; l.s=l.spawnS; l.pos=l.spawn; l.heading=l.spawnHeading;
        l.nextRest=now+RandomRange(6,12); l.nextSearch=now; l.nextSwitch=now+10;
        PlaySlai(l,false); hold();
        return;
    }
    if (now < l.hurtUntil) { hold(); return; }
    if (!l.path && now >= l.nextSearch)
    {
        l.nextSearch=now+1;
        StartSlaiPath(psw,l);
    }
    const CRV* c=PcrvFromLo(l.path);
    if (!c || c->as.size()<2 || !(c->as.back()-c->as.front()>80))
    { l.path=nullptr; PlaySlai(l,false); hold(); return; }
    auto sample = [&](float s, glm::vec3& q, glm::vec3& dir) {
        EvalCrvS(c,s,&q,&dir);
        if (!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.z)) return false;
        glm::vec3 floor=q; if (FDropOnGround(&floor,60,-100,false)) q=floor;
        return true;
    };
    glm::vec3 q(0), tangent(0);
    if (!sample(l.s,q,tangent)) return;
    if (!l.hasSpawn) { l.hasSpawn=true; l.spawn=q; l.spawnS=l.s; l.spawnPath=l.path; l.spawnHeading=l.heading; l.nextRest=now+RandomRange(6,12); l.nextSwitch=now+10; }
    l.pos=q; l.fPosition=true;
    LO* player=PloPoCurrent(psw);
    const bool carm=player && player!=l.plo && FPcCarm(player) && FCidDerivedFrom(player->Cid(),0x30);
    glm::vec3 pp=carm ? PosAloCenter(static_cast<ALO*>(player)) : glm::vec3(0);
    float d=carm ? Distance(pp,l.pos+glm::vec3(0,0,65)) : 1e10f;
    const bool seen=carm && d<1400 && SlaiClearLine(l.pos+glm::vec3(0,0,65),pp);
    if (seen && d<300) l.fleeing=true;
    else if (d>850 || !seen) l.fleeing=false;
    if (l.fleeing) { l.attackUntil=0; l.attackTarget=nullptr; l.restUntil=0; }
    if (now<l.attackUntil && !l.fleeing)
    {
        if (LiveGuard(l.attackTarget)) FaceSlai(l,PosAloCenter(static_cast<ALO*>(l.attackTarget)),dt);
        if (!l.struck && now>=l.strikeAt) { l.struck=true; StrikeSlai(l); }
        hold(); return;
    }
    l.attackTarget=nullptr;
    if (!l.fleeing && (!seen || d>500) && now>=l.nextAttack && !l.attacks.empty())
    {
        if (LO* target=NearbyGuard(l.pos))
        {
            l.attackTarget=target; l.struck=false; l.restUntil=0;
            ASEG* clip=l.attacks[std::uniform_int_distribution<size_t>(0,l.attacks.size()-1)(s_rng)];
            const float duration=ClipTime(clip,0.8f);
            l.attackUntil=now+duration; l.strikeAt=now+duration*0.45f; l.nextAttack=l.attackUntil+0.8f;
            FaceSlai(l,PosAloCenter(static_cast<ALO*>(target)),dt); RestartClip(l,clip); hold(); return;
        }
    }
    float speed = l.run ? c_sSlaiTravel : c_sSlaiWalk;
    bool move=true;
    const float first=c->as.front(),last=c->as.back();
    auto wrap=[&](float s){return c->fClosed?SWrapCrv(c,s):std::clamp(s,first,last);};
    if (l.fleeing || seen)
    {
        l.restUntil=0; l.nextRest=now+RandomRange(6,12);
        glm::vec3 a(0),b(0),dummy(0);
        if (!sample(wrap(l.s+100),a,dummy)||!sample(wrap(l.s-100),b,dummy)) return;
        float da=Distance(a,pp),db=Distance(b,pp);
        if (l.fleeing) { l.dir=da>db?1:-1; speed=l.run ? c_sSlaiRun : c_sSlaiWalk; move=std::max(da,db)>Distance(q,pp)+1; }
        else if (d>700) { l.dir=da<db?1:-1; if (d<850) speed=c_sSlaiWalk; move=std::min(da,db)<Distance(q,pp)-1; }
        else move=false;
        if (!move && l.fleeing && now>=l.nextSwitch)
        {
            float ns=0; LO* other=FindPatrol(psw,l.pos,ns,l.path,55);
            l.nextSwitch=now+1;
            if (other) { l.path=other; l.s=ns; hold(); return; }
        }
        if (!move) FaceSlai(l,pp,dt);
    }
    else
    {
        if (now>=l.nextRest && now>=l.restUntil)
        {
            l.restUntil=now+RandomRange(2,5); l.nextRest=l.restUntil+RandomRange(6,14);
            l.restClip=l.idles.empty()?l.idle:l.idles[std::uniform_int_distribution<size_t>(0,l.idles.size()-1)(s_rng)];
        }
        if (now<l.restUntil) { PlayClip(l,l.restClip); hold(); return; }
        if (now>=l.nextSwitch)
        {
            float ns=0; LO* other=FindPatrol(psw,l.pos,ns,l.path,55);
            l.nextSwitch=now+RandomRange(8,16);
            if (other) { l.path=other; l.s=ns; PlaySlai(l,false); hold(); return; }
        }
    }
    if (move)
    {
        l.s+=float(l.dir)*speed*dt;
        if (c->fClosed) l.s=SWrapCrv(c,l.s);
        else if(l.s>=last){l.s=last;l.dir=-1;} else if(l.s<=first){l.s=first;l.dir=1;}
        if(!sample(l.s,q,tangent)) return;
        if (tangent.x*tangent.x+tangent.y*tangent.y>0.000001f) FaceSlai(l,l.pos+glm::vec3(tangent.x*l.dir,tangent.y*l.dir,0)*100.0f,dt);
        l.pos=q;
    }
    hold();
    const bool running = move && speed > c_sSlaiWalk && l.run;
    PlayClip(l, move ? (running ? l.run : l.walk) : l.idle, true,
             running ? speed / c_sSlaiRun : 1.0f);
}

// after the world's load, before its start: each guest's copy made (started with the world's own objects) and placed
void OnLoaded(SW* psw)
{
    s_alive.clear();
    if (!psw) return;
    s_serial = psw->serial;
    const std::string world = WorldNameOf(g_transition.iwrecCur);
    const auto guests = GuestsOf(world);
    s_alive.reserve(guests.size());
    for (const cane::mods::Guest* pg : guests)
    {
        const int iwrec = IwrecFrom(*pg);
        if (iwrec < 0)
        {
            std::fprintf(stderr, "mods: guest %s of %s: no level \"%s\"\n", pg->id.c_str(), world.c_str(), pg->from.c_str());
            continue;
        }
        if (PckwsKeptWorld(iwrec) != PCKWS_Ready)
        {
            std::fprintf(stderr, "mods: guest %s of %s: %s did not load\n", pg->id.c_str(), world.c_str(), pg->from.c_str());
            continue;
        }
        LO* ptmpl = PloKeptFind(iwrec, pg->object, 0);
        if (!ptmpl)
        {
            std::fprintf(stderr, "mods: guest %s of %s: %s has no object %d\n", pg->id.c_str(), world.c_str(), pg->from.c_str(), pg->object);
            continue;
        }
        LO* plo = PloPcCloneIntoWorld(psw, ptmpl, true);
        if (!plo) continue;
        s_alive.push_back({pg, plo});
        s_alive.back().fSlai = IsSlai(*pg, plo);
        std::printf("mods: guest %s in %s: %s's object %d (cid 0x%X) at %.0f %.0f %.0f\n", pg->id.c_str(), world.c_str(), pg->from.c_str(), pg->object,
                    plo->Cid(), pg->at[0], pg->at[1], pg->at[2]);
    }
}

// after the start: a guard's patrol (a path of this world, walked along and back, again and again)
void OnStarted(SW* psw)
{
    if (!psw || psw->serial != s_serial) return;
    for (LIVE& l : s_alive)
    {
        if (l.fSlai)
        {
            if (l.plo->fRemoved || !l.plo->fInWorld) AddLo(psw, l.plo);
            SetLoSt(psw, l.plo, 1);
            ALO* palo = static_cast<ALO*>(l.plo);
            if (SOST* ps = Psost(palo)) ps->dvGravity = glm::vec3(0.0f);
            const int iwrec = IwrecFrom(*l.pg);
            l.walk = dynamic_cast<ASEG*>(PloKeptFind(iwrec, 277, 0x6D));
            l.idle = dynamic_cast<ASEG*>(PloKeptFind(iwrec, 4576, 0x6D));
            auto clip = [&](int oid) { return dynamic_cast<ASEG*>(PloKeptFind(iwrec, oid, 0x6D)); };
            l.run=clip(126); l.hurt=clip(222); l.defeat=clip(231);
            for(int oid : {4576,4560,5543,5546,5552,5539}) if(auto* a=clip(oid)) l.idles.push_back(a);
            for(int oid : {291,290,288,287}) if(auto* a=clip(oid)) l.attacks.push_back(a);
            if (ACTOR* pa=PactorFromLo(l.plo)) { SetDmgCapacity(pa,400); SetDmgCounter(pa,&l.hp); }
            if (l.pg->patrol >= 0) Place(l.plo, *l.pg);
            // The automatic guest uses a nearby patrol, never the manifest's dummy origin.
            STEP* step = PstepFromLo(l.plo);
            l.pos = step ? glm::vec3(step->posFloor) : PosAloCenter(palo);
            l.heading = step ? step->radHeading : glm::radians(l.pg->heading);
            l.fPosition = true;
            l.nextSearch = g_clock.t;
            std::printf("Slai: independent guest; walk %d idle %d\n", l.walk ? l.walk->oid : -1, l.idle ? l.idle->oid : -1);
            if (!l.walk) std::fprintf(stderr, "Slai: missing ASEG 277 in source world; stays idle\n");
            PlaySlai(l, false);
            continue;
        }
        Place(l.plo, *l.pg);   // (after the start: placed before, the start's records put it back where its template stands)
        if (FCidDerivedFrom(l.plo->Cid(), 0x34))
        {
            // Sly, Bentley or Murray as a character of the level (not the player): no gravity, held at its place, its idle
            // state machine's first animation looped (the player's +0x1028 part, gang.cpp psmIdle; else its +0x6F0 one)
            ALO* palo = static_cast<ALO*>(l.plo);
            if (SOST* ps = Psost(palo)) ps->dvGravity = glm::vec3(0.0f);
            l.fHold = true;
            ASEG* paseg = nullptr;
            for (const auto& [ib, ppart] : palo->apart)
                if ((ib == 0x1028 || ib == 0x6F0) && ppart && ppart->Cid() == 0x6F)
                    for (const SM::SMT& t : static_cast<SM*>(ppart)->asmt)
                        if (t.paseg && !paseg) paseg = t.paseg;
            if (paseg) PasegaEnsureAseg(paseg, palo, 0, 0.0f, 1.0f, 8);
            if (FPrint()) std::printf("mods: guest %s stands as a character (idle %d)\n", l.pg->id.c_str(), paseg ? paseg->oid : -1);
            continue;
        }
        GUARD* pgd = FCidDerivedFrom(l.plo->Cid(), 0x38) ? PguardFromLo(l.plo) : nullptr;
        if (!pgd) continue;
        if (l.pg->patrol < 0)
        {
            // (its own level's patrol names that level's paths: none here, it keeps its post)
            SetGuardPatrol(pgd, nullptr);
            continue;
        }
        LO* ppath = PloFindSwObjectByOid(psw, l.pg->patrol);
        if (!ppath)
        {
            std::fprintf(stderr, "mods: guest %s: no path %d in this world\n", l.pg->id.c_str(), l.pg->patrol);
            SetGuardPatrol(pgd, nullptr);
            continue;
        }
        auto* pl = static_cast<CMDLIST*>(PloNew(0x71, psw, nullptr, 0x2CB, -1));
        if (!pl) continue;
        // (label 1, along the path, back along it, back to label 1: the levels' own patrols' commands, ai.h CMD)
        PcmdAppend(pl, 9, 1);
        for (int fForward : {1, 0})
            if (CMD* pc = PcmdAppend(pl, 6, l.pg->patrol))
            {
                pc->n8 = fForward;
                pc->nC = -1;
            }
        PcmdAppend(pl, 10, 1);
        PostLoadCmdlist(pl);
        SetGuardPatrol(pgd, pl);
        if (FPrint()) std::printf("mods: guest %s walks along path %d\n", l.pg->id.c_str(), l.pg->patrol);
    }
}

// (SLY2N_GUESTPRINT: each guest's place every second)
void OnStep(SW* psw, bool)
{
    if (!psw || psw->serial != s_serial) return;
    for (LIVE& l : s_alive)
    {
        if (l.fSlai) StepSlai(psw, l);
        else if (l.fHold) Place(l.plo, *l.pg);
    }
    if (!FPrint()) return;
    static int s_c = 0;
    if (++s_c % 60 != 0) return;
    for (const LIVE& l : s_alive)
        if (FCidDerivedFrom(l.plo->Cid(), 0x30))
        {
            const glm::vec3 p = PosAloCenter(static_cast<ALO*>(l.plo));
            std::printf("mods: guest %s at %.0f %.0f %.0f%s%s\n", l.pg->id.c_str(), p.x, p.y, p.z, l.plo->fInWorld ? "" : " (out of the world)",
                        l.plo->fRemoved ? " (removed)" : "");
        }
}

struct INIT
{
    INIT()
    {
        PCWORLDHOOKS h;
        h.pfnLoaded = OnLoaded;
        h.pfnStarted = OnStarted;
        h.pfnStep = OnStep;
        RegisterPcWorldHooks(h);
    }
} s_init;
}  // namespace

void RequestModGuestWorlds(const uint8_t* abChk)
{
    const auto& awrec = AwrecWorlds();
    int iwrec = -1;
    for (std::size_t i = 0; i < awrec.size(); ++i)
        if (std::memcmp(awrec[i].abRecord, abChk, 64) == 0) iwrec = int(i);
    for (const cane::mods::Guest* pg : GuestsOf(WorldNameOf(iwrec)))
        if (const int iwrecFrom = IwrecFrom(*pg); iwrecFrom >= 0) RequestPcKeptWorld(iwrecFrom);
}

bool FAddGuestArg(const std::string& arg)
{
    // FROM:OBJECT@X,Y,Z[,HEADING][:PATROL]
    const std::size_t at = arg.find('@');
    const std::size_t colon = arg.rfind(':', at);
    if (at == std::string::npos || colon == std::string::npos) return false;
    cane::mods::Guest g;
    g.world = "*";
    g.from = arg.substr(0, colon);
    g.object = std::atoi(arg.c_str() + colon + 1);
    std::string rest = arg.substr(at + 1);
    if (const std::size_t c2 = rest.find(':'); c2 != std::string::npos)
    {
        g.patrol = std::atoi(rest.c_str() + c2 + 1);
        rest.resize(c2);
    }
    float v[4] = {0, 0, 0, 0};
    const int n = std::sscanf(rest.c_str(), "%f,%f,%f,%f", &v[0], &v[1], &v[2], &v[3]);
    if (n < 3) return false;
    for (int i = 0; i < 3; ++i) g.at[i] = v[i];
    g.heading = v[3];
    g.id = "cli-" + std::to_string(g_disc.Aguest().size());
    g.name = g.from + " " + std::to_string(g.object);
    g_disc.AddGuest(g);
    return true;
}

int CModGuestsLive() { return int(s_alive.size()); }
LO* PloModGuestLive(int i) { return i >= 0 && i < int(s_alive.size()) ? s_alive[std::size_t(i)].plo : nullptr; }

// Used before the native player initialization: the guest is not the player.
bool FModSlaiGuest(const LO* plo)
{
    for (const LIVE& l : s_alive) if (l.fSlai && l.plo == plo) return true;
    return false;
}
bool FModSlaiTargetable(const LO* plo)
{
    for (const LIVE& l : s_alive) if (l.fSlai && l.plo == plo)
        return plo && plo->fInWorld && !plo->fRemoved && l.hasSpawn && l.hp > 0;
    return false;
}
int* PnModSlaiHealth(const LO* plo)
{
    for (LIVE& l : s_alive) if (l.fSlai && l.plo == plo) return &l.hp;
    return nullptr;
}

// Called before native player damage/mission transitions. Slai owns his reaction and respawn.
bool FModSlaiAcceptDamage(LO* plo, const CTMSG* hit)
{
    if (!hit || !hit->ploSender || hit->ploSender == plo || hit->nState >= 0 || hit->grfFilter <= 1 || hit->grfFilter == 0x100000 || hit->grfFilter == 0x80000) return false;
    for (LIVE& l : s_alive) if (l.fSlai && l.plo == plo)
    {
        if (!l.hasSpawn || l.hp <= 0 || g_clock.t < l.nextHit || FGamePausedForAi()) return false;
        l.hp -= int(std::min<int64_t>(l.hp,-int64_t(hit->nState)));
        if (std::getenv("SLY2N_SLAIGUARD"))
            std::printf("slai hit: source %d, damage %d, hp %d\n", hit->ploSender->oid, hit->nState, l.hp);
        l.nextHit=g_clock.t+1.0f; l.attackUntil=0; l.attackTarget=nullptr; l.restUntil=0;
        if (l.hp <= 0) { l.respawnAt=g_clock.t+6.0f; RestartClip(l,l.defeat); }
        else { l.hurtUntil=g_clock.t+ClipTime(l.hurt,0.6f); RestartClip(l,l.hurt); }
        return true;
    }
    return false;
}
