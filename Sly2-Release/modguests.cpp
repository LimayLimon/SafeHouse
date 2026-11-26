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
#include "bone.h"
#include "cls.h"
#include "disc.h"
#include "guard.h"
#include "frame.h"
#include "damage.h"
#include "bsp.h"
#include "pccharacter.h"
#include "projectile.h"
#include "pickup.h"
#include <random>
#include <deque>
#include <unordered_map>
#include "lo.h"
#include "modworld.h"
#include "pcloader.h"
#include "so.h"
#include "step.h"
#include "stepphys.h"
#include "sly.h"
#include "sw.h"
#include "transition.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <string>
#include <vector>

namespace
{
struct SlaiNavNode { glm::vec3 pos; int parent; };
struct LIVE
{
    const cane::mods::Guest* pg = nullptr;
    LO* plo = nullptr;
    bool fHold = false;   // (a player class: kept at its place)
    bool fSlai = false;  // opt-in: independent Sly guest, never the player
    int hp = c_nSlaiMaxHP;
    int coins = 0;
    glm::vec3 pickupGoal{0};
    bool pickupHeal=false, pickupHealOnly=false;
    float nextPickupSearch=0, pickupUntil=0, pickupStarted=0, pickupSkipUntil=0;
    glm::vec3 pickupSkipPos{0};
    const char* state = "spawn";
    glm::vec3 blockedPos{0}, freeGoal{0};
    float blockedUntil=0, freeUntil=0, nextLocal=0, nextPad=0, nextRecovery=0;
    LO* path = nullptr;
    ASEG* walk = nullptr;
    ASEG* idle = nullptr;
    ASEG* run = nullptr;
    ASEG* jumpClip = nullptr;
    ASEG* doubleJumpClip = nullptr;
    ASEG* hurt = nullptr;
    ASEG* shock = nullptr;
    ASEG* combatIdle = nullptr;
    ASEG* combatWalk = nullptr;
    ASEG* defeat = nullptr;
    ASEG* playing = nullptr;
    std::vector<ASEG*> idles, attacks;
    ASEG* restClip = nullptr;
    LO* attackTarget = nullptr;
    float lastGuardSeen = 0, nextCombat = 0;
    float restUntil = 0, nextRest = 0, nextSwitch = 0;
    float hurtUntil = 0, respawnAt = 0, nextHit = 0;
    float attackUntil = 0, strikeAt = 0, nextAttack = 0;
    bool struck = false, fleeing = false;
    // Social activity is separate from mood and never overrides danger.
    LO* socialPlayer = nullptr;
    bool following = false, followHolding = false, visiting = false;
    float lastPlayerSeen = 0, holdUntil = 0, nextVisit = 0;
    float visitDeadline = 0, visitEnd = 0, socialCooldown = 0;
    bool hasSpawn = false;
    glm::vec3 spawn{0.0f};
    float spawnS = 0, spawnHeading = 0;
    LO* spawnPath = nullptr;
    float s = 0.0f;
    int dir = 1;
    float heading = 0.0f;
    float nextSearch = 0.0f;
    float routeSince = 0, lastProgress = 0, nextProgressCheck = 0, nextDirection = 0;
    glm::vec3 progressPos{0.0f}, safePos{0.0f};
    bool hasSafePos = false;
    float nextDrySample=0, waterSince=0, nextWaterTry=0;
    bool waterJump=false;
    // Independent bounded local navigation graph, built only when escape steering fails.
    std::vector<SlaiNavNode> navNodes;
    std::deque<glm::vec3> navRoute;
    size_t navCursor=0;
    float navNext=0, navCooldown=0, navStarted=0;
    std::deque<LO*> recentPaths;
    float actualSpeed = 0.0f;
    // A bounded jump owns locomotion until actual touchdown, never the curve height.
    bool jumping = false;
    float jumpStarted = 0, jumpDeadline = 0, nextJump = 0;
    // A high jump has one physical launch and one bounded mid-air impulse.
    bool roofSecondPending = false, roofFlight = false;
    float roofSecondAt = 0, nextRoof = 0;
    glm::vec3 roofApex{0};
    LO* jumpPath = nullptr;
    glm::vec3 jumpGoal{0.0f};
    float jumpS = 0;
    int jumpDir = 1;
    float nextPadScan = 0;
    std::vector<OID> reportedPads;
    glm::vec3 pos{0.0f};
    bool fPosition = false;
};
std::deque<LIVE> s_alive; // addresses of individual health counters remain stable
std::vector<const cane::mods::Guest*> s_pendingSlai;
const cane::mods::Guest* s_spawningSlai = nullptr;
LO* s_readyPlayer = nullptr;
float s_readyAt = 0.0f;
uint32_t s_serial = 0;
std::mt19937 s_rng{0x534C4149u};
constexpr float c_sSlaiWalk = 350.0f;
constexpr float c_sSlaiRun = 700.0f;
constexpr float c_sSlaiDetect = 2500.0f;
constexpr float c_sSlaiObserve = 1400.0f;
constexpr float c_sSlaiSafe = 3000.0f;
constexpr float c_sSlaiFollowStop = 900.0f, c_sSlaiFollowResume = 1500.0f;
// An unsolicited visit stops just outside Carmelita's existing discovery radius.
constexpr float c_sSlaiVisitStop = 1500.0f, c_sSlaiVisitResume = 2100.0f;
constexpr float c_sSlaiVisitRange = 6000.0f;
constexpr float c_gSlaiVisual = 0.70710678f; // 90 degree horizontal FOV, character heading, not camera
float RandomRange(float a, float b) { return std::uniform_real_distribution<float>(a,b)(s_rng); }
void ResetSocial(LIVE& l,float now,bool pause)
{
    l.following=false;l.followHolding=false;l.visiting=false;
    l.holdUntil=0;l.lastPlayerSeen=0;l.visitDeadline=0;l.visitEnd=0;
    l.nextVisit=now+RandomRange(60,120);
    l.socialCooldown=pause?now+RandomRange(10,20):0;
    l.freeUntil=0;l.nextLocal=now;
}
float SlaiExploreTime() {
    const float r=RandomRange(0,1);
    return r<0.20f?RandomRange(5,12):r<0.75f?RandomRange(15,38):RandomRange(40,75);
}
float SlaiRestTime() {
    const float r=RandomRange(0,1);
    return r<0.45f?RandomRange(2,5):r<0.90f?RandomRange(6,13):RandomRange(14,24);
}
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
        if (FModSlaiGuest(plo)) { SetStepRotation(pstep,m); SetStepPos(pstep,pos-m*glm::vec3(pstep->vecBase)); }
        else { SetStepPos(pstep,pos-m*glm::vec3(pstep->vecBase)); SetStepRotation(pstep,m); }
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

bool WantsSlai(const cane::mods::Guest& g)
{
    return g.id == "slai" || g.id.rfind("slai-", 0) == 0 || g.name == "Slai";
}
bool IsSlai(const cane::mods::Guest& g, const LO* plo)
{
    return FCidDerivedFrom(plo->Cid(), 0x35) && WantsSlai(g);
}

// Native patrol path curves guide a physical STEP actor, not a GUARD cast.
std::vector<LO*> PatrolPaths(SW* psw)
{
    std::vector<LO*> paths;
    auto scan = [&](const CMDLIST* list) {
        if (!list) return;
        for (const auto& cmd : list->acmd) if (cmd && cmd->op == 6)
        {
            LO* path=PloFindSwObjectByOid(psw,cmd->oid);
            const CRV* c=PcrvFromLo(path);
            if (path && c && c->as.size()>=2 && c->as.back()-c->as.front()>80 &&
                std::find(paths.begin(),paths.end(),path)==paths.end()) paths.push_back(path);
        }
    };
    for (const auto& obj:psw->aploAll)
    {
        LO* o=obj.get(); if(!o) continue;
        if(o->Cid()==0x71 && o->oid==0x2CB) scan(static_cast<CMDLIST*>(o));
        if(FCidDerivedFrom(o->Cid(),0x38)) if(GUARD* g=PguardFromLo(o)) scan(g->pcmdlistPatrol);
    }
    return paths;
}
LO* FindPatrol(SW* psw,const glm::vec3& anchor,float& bestS,LO* exclude=nullptr,float maxDistance=std::numeric_limits<float>::max())
{
    LO* best=nullptr; float bestDist=maxDistance;
    for(LO* path:PatrolPaths(psw))
    {
        if(path==exclude) continue;
        const CRV* c=PcrvFromLo(path);
        glm::vec3 q(0); float s=0;
        NearestCrvAll(c,anchor,&q,nullptr,nullptr,&s);
        if(!FDropOnGround(&q,100,-250,false)) continue;
        const float d=glm::length(q-anchor);
        if(std::isfinite(s)&&std::isfinite(d)&&d<bestDist){best=path;bestDist=d;bestS=s;}
    }
    return best;
}

// Read-only discovery of potential upward bounce-pad links. Symbol 1088 is
// the destination used by the authored bounce-pad scripts (pad.cpp). A PAD
// alone is not proof that its script launches actors; never run its events here.
void ScanSlaiPads(SW* psw, LIVE& l)
{
    if (g_clock.t < l.nextPadScan) return;
    l.nextPadScan = g_clock.t + 5.0f;
    for (const auto& obj : psw->aploAll)
    {
        LO* pad = obj.get();
        if (!pad || pad->Cid() != 0x53 || !pad->fInWorld || pad->fRemoved || !pad->pframe) continue;
        CRef* ref = pad->pframe->PrefFindBinding(1088, 1);
        if (!ref || ref->m_tagk != TAGK_Vector) continue;
        const glm::vec3 from = PosAloCenter(static_cast<ALO*>(pad));
        const glm::vec3 to = ref->GetVector();
        if (!std::isfinite(to.x) || !std::isfinite(to.y) || !std::isfinite(to.z) || to.z <= from.z + 40.0f) continue;
        if (std::find(l.reportedPads.begin(), l.reportedPads.end(), pad->oid) != l.reportedPads.end()) continue;
        float s = 0.0f;
        LO* landingPath = FindPatrol(psw, to, s, nullptr, 250.0f);
        // Recheck later: streamed-in guard paths might not be available yet.
        if (!landingPath) continue;
        l.reportedPads.push_back(pad->oid);
        std::printf("Slai: candidate upward pad %d from %.0f %.0f %.0f to %.0f %.0f %.0f landing path %d\n",
                    pad->oid, from.x, from.y, from.z, to.x, to.y, to.z, landingPath->oid);
    }
}

void PlayClip(LIVE& l, ASEG* want, bool loop = true, float rate = 1.0f)
{
    if (!want) want = l.idle;
    if (!want) return;
    ALO* palo = static_cast<ALO*>(l.plo);
    ASEGA* previous = l.playing && l.playing != want ? PasegaFindAseg(l.playing,palo) : nullptr;
    if (ASEGA* a = PasegaFindAseg(want, palo))
    {
        if (previous) RetractAsega(previous);
        l.playing=want;
        if (std::abs(a->svtLocal - rate) > 0.035f) SetAsegaSpeed(a, rate);
        return;
    }
    if (ASEGA* a = PasegaApplyAseg(want, palo, 0.0f, rate, loop ? 9 : 33))
    {
        SetAsegaPriority(a, 20);
        if (previous) RetractAsega(previous);
        l.playing=want;
        SetAsegaSeekTime(a, g_clock.t + 0.12f);
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
// Actual feet position from the root transform, rather than stale floor contact or a curve.
glm::vec3 SlaiFeet(LO* plo)
{
    ALO* a=static_cast<ALO*>(plo);
    if(STEP* step=PstepFromLo(plo))
        if(const XF* x=PxfObj(a)) return glm::vec3(x->matWorld*glm::vec4(glm::vec3(step->vecBase),1));
    return PosAloCenter(a)-glm::vec3(0,0,75);
}
// A short ground-tested transition: reject gaps, vertical discontinuities and walls.
bool SlaiWetPoint(const glm::vec3& q)
{
    const glm::vec3 a=q+glm::vec3(0,0,3), b=q+glm::vec3(0,0,75);
    for(OXA* ox:g_physw.apoxaActive){
        if(!ox||!ox->pso||ox->pso->Cid()!=0x59||q.x<ox->posMin.x||q.x>ox->posMax.x||q.y<ox->posMin.y||q.y>ox->posMax.y||b.z<ox->posMin.z||a.z>ox->posMax.z)continue;
        for(SOE* e:ox->apsoe){if(!e||e->f50)continue;LSG hits[8];if(ClipSegmentEntry(e,a,b,8,hits)>0)return true;}
    }
    return false;
}
bool GroundConnection(const glm::vec3& from,const glm::vec3& to)
{
    glm::vec3 last=from;
    const float dist=glm::length(glm::vec2(to-from));
    const int count=std::clamp(int(std::ceil(dist/30.0f)),1,12);
    for(int i=1;i<=count;++i)
    {
        glm::vec3 q=glm::mix(from,to,float(i)/count);
        q.z=last.z;
        if(!FDropOnGround(&q,50,-70,false)||std::abs(q.z-last.z)>35||SlaiWetPoint(q)) return false;
        if(!SlaiClearLine(last+glm::vec3(0,0,65),q+glm::vec3(0,0,65))) return false;
        // A center ray alone can pass a corner even while the body gets caught.
        const glm::vec2 travel(q-last); const float len=glm::length(travel);
        if(len>1){const glm::vec2 side(-travel.y/len,travel.x/len);
            for(float width:{-22.0f,22.0f}){const glm::vec3 offset(side*width,55);if(!SlaiClearLine(last+offset,q+offset))return false;}}
        last=q;
    }
    return std::abs(last.z-to.z)<65;
}
void SlaiDrive(LIVE& l,const glm::vec3& desired,float speed,float dt)
{
    STEP* step=PstepFromLo(l.plo); if(!step)return;
    ALO* a=step->palo;
    glm::vec3 v=desired-l.pos; v.z=0;
    const float len=glm::length(v);
    if(len>1 && speed>0)
    {
        FaceSlai(l,desired,dt);
        v*=std::min(speed,len/std::max(dt,0.001f))/len;
    }
    else
    {
        v=glm::vec3(0);
        if(step->pxpFloor) { const auto old=VelAloPos(a); SetAloVelocityTo(a,glm::vec3(0,0,old.z)); }
    }
    // STEP's floor solver uses negative desired velocity in the heading frame (as GUARD does).
    // Keep vertical integration, floor contacts, collision response and angular braking native.
    step->nMotion=0; step->fNoContacts=0;
    SetStepPhysHeading(a,l.heading);
    step->vecVel=-glm::transpose(step->matHeading)*v;
    WakeAlo(a);
}
// Short local steering: use actual floor samples rather than forcing a point on a curve.
bool LocalGoal(LIVE& l,const glm::vec3& desired,bool flee,bool explore,glm::vec3& out)
{
    glm::vec2 forward=glm::vec2(desired-l.pos);
    if(flee)forward=-forward;
    if(explore||glm::length(forward)<1)forward=glm::vec2(std::cos(l.heading),std::sin(l.heading));
    else forward=glm::normalize(forward);
    float best=-1e20f;bool found=false;
    for(float angle:{0.0f,0.52f,-0.52f,1.05f,-1.05f,1.57f,-1.57f,2.35f,-2.35f,3.14f})
    for(float length:{180.0f,95.0f})
    {
        const glm::vec2 v(forward.x*std::cos(angle)-forward.y*std::sin(angle),forward.x*std::sin(angle)+forward.y*std::cos(angle));
        glm::vec3 q=l.pos+glm::vec3(v*length,0);
        if(!FDropOnGround(&q,60,-80,false)||!GroundConnection(l.pos,q))continue;
        if(g_clock.t<l.blockedUntil && Distance(q,l.blockedPos)<100 && Distance(q,l.blockedPos)<=Distance(l.pos,l.blockedPos)+20)continue;
        float score=glm::dot(v,forward)*length-std::abs(angle)*12;
        if(!explore)score+=(flee?1:-1)*(Distance(q,desired)-Distance(l.pos,desired));
        if(score>best){best=score;out=q;found=true;}
    }
    return found;
}
bool SamplePath(const CRV* c,float s,glm::vec3& q)
{
    if(!c || c->as.size()<2)return false;
    glm::vec3 tangent(0);
    s=c->fClosed?SWrapCrv(c,s):std::clamp(s,c->as.front(),c->as.back());
    EvalCrvS(c,s,&q,&tangent);
    return std::isfinite(q.x)&&std::isfinite(q.y)&&std::isfinite(q.z)&&FDropOnGround(&q,100,-250,false)&&!SlaiWetPoint(q);
}
float BoundS(const CRV* c,float s){return c->fClosed?SWrapCrv(c,s):std::clamp(s,c->as.front(),c->as.back());}
void RememberPath(LIVE& l,LO* path)
{
    if(!path)return;
    l.recentPaths.erase(std::remove(l.recentPaths.begin(),l.recentPaths.end(),path),l.recentPaths.end());
    l.recentPaths.push_back(path);
    if(l.recentPaths.size()>5)l.recentPaths.pop_front();
}
bool SwitchPath(SW* psw,LIVE& l,const glm::vec3* away,bool force)
{
    LO* best=nullptr;float bestS=0,bestScore=-1e20f;int bestDir=1;
    for(LO* path:PatrolPaths(psw))
    {
        if(path==l.path)continue;
        const CRV* c=PcrvFromLo(path);float ns=0;glm::vec3 raw(0),q(0);
        NearestCrvAll(c,l.pos,&raw,nullptr,nullptr,&ns);
        if(!std::isfinite(ns)||glm::length(glm::vec2(raw-l.pos))>150 || !SamplePath(c,ns,q))continue;
        const float dist=glm::length(q-l.pos);
        if(dist>150||std::abs(q.z-l.pos.z)>55||!GroundConnection(l.pos,q))continue;
        const bool recent=std::find(l.recentPaths.begin(),l.recentPaths.end(),path)!=l.recentPaths.end();
        if(recent&&!force&&!away)continue;
        for(int dir:{1,-1})
        {
            glm::vec3 ahead(0);if(!SamplePath(c,ns+dir*100,ahead)||glm::length(glm::vec2(ahead-q))<15||!GroundConnection(q,ahead))continue;
            float score=-dist+(recent?-150:180);
            if(away)score+=2*(glm::length(ahead-*away)-glm::length(l.pos-*away));
            else score+=RandomRange(0,25);
            if(score>bestScore){best=path;bestS=ns;bestDir=dir;bestScore=score;}
        }
    }
    if(!best)return false;
    RememberPath(l,l.path);l.path=best;l.s=bestS;l.dir=bestDir;l.routeSince=g_clock.t;l.nextDirection=g_clock.t+1.5f;
    if(FPrint())std::printf("Slai: switched to path %d at %.1f\n",best->oid,bestS);
    return true;
}
// Test candidate landings using the same static floor/obstacle queries as walking.
// This intentionally handles only modest ledges and downward transfers, not arbitrary roof climbing.
struct SlaijumpPlan
{
    LO* path = nullptr;
    glm::vec3 target{0}, velocity{0};
    float s = 0, flight = 0;
    int dir = 1;
};
bool PlanSlaiLanding(LIVE& l, const glm::vec3& q, LO* path, float s, int dir, SlaijumpPlan& out)
{
    const glm::vec3 delta=q-l.pos;
    const float span=glm::length(glm::vec2(delta));
    if(span<65 || span>350 || delta.z>110 || delta.z<-550 || SlaiWetPoint(q))return false;
    // A normal walking connection does not need a jump.
    if(GroundConnection(l.pos,q))return false;
    for(glm::vec2 offset:{glm::vec2(22,0),glm::vec2(-22,0),glm::vec2(0,22),glm::vec2(0,-22)})
    {
        glm::vec3 test=q+glm::vec3(offset,0);
        if(!FDropOnGround(&test,35,-35,false)||std::abs(test.z-q.z)>25||SlaiWetPoint(test))return false;
    }
    // Landing may be off-path, but must have an exit on verified ground.
    glm::vec3 ahead(0); bool exit=false;
    if(path) { const CRV* c=PcrvFromLo(path); exit=SamplePath(c,s+dir*65,ahead)&&glm::length(glm::vec2(ahead-q))>=15&&GroundConnection(q,ahead); }
    else for(const glm::vec2 v:{glm::vec2(65,0),glm::vec2(-65,0),glm::vec2(0,65),glm::vec2(0,-65)}) { ahead=q+glm::vec3(v,0); if(FDropOnGround(&ahead,35,-35,false)&&GroundConnection(q,ahead)){exit=true;break;} }
    if(!exit)return false;
    const SOST* st=Psost(static_cast<ALO*>(l.plo));
    if(!st||!std::isfinite(st->dvGravity.z)||st->dvGravity.z>=-1)return false;
    const float g=-st->dvGravity.z;
    // Choose a small arc for a step up or down. Apply only the initial velocity;
    // native STEP physics, gravity and collision response integrate the flight.
    const float rise=std::max(0.0f,delta.z)+105.0f;
    const float vz=std::sqrt(2*g*rise);
    const float t=(vz+std::sqrt(std::max(0.0f,vz*vz-2*g*delta.z)))/g;
    if(!std::isfinite(t)||t<0.2f||t>1.7f||span/t>c_sSlaiRun)return false;
    const glm::vec3 vel(delta.x/t,delta.y/t,vz);
    glm::vec3 last=l.pos;
    const int count=std::clamp(int(std::ceil(t/0.05f)),8,36);
    for(int i=1;i<=count;++i)
    {
        const float ti=t*float(i)/count;
        const glm::vec3 at=l.pos+vel*ti-glm::vec3(0,0,0.5f*g*ti*ti);
        for(float z:{24.0f,90.0f,145.0f})
            if(!SlaiClearLine(last+glm::vec3(0,0,z),at+glm::vec3(0,0,z)))return false;
        last=at;
    }
    out={path,q,vel,path?BoundS(PcrvFromLo(path),s):0.0f,t,dir};return true;
}
bool PlanSlaiJump(LIVE& l, LO* path, float s, int dir, SlaijumpPlan& out)
{
    glm::vec3 q(0); return SamplePath(PcrvFromLo(path),s,q)&&PlanSlaiLanding(l,q,path,s,dir,out);
}
bool TrySlaiJump(SW* psw,LIVE& l,const glm::vec3* interest,bool away)
{
    STEP* st=PstepFromLo(l.plo);
    const float now=g_clock.t;
    if(!st||!st->pxpFloor||!l.jumpClip||now<l.nextJump)return false;
    l.nextJump=now+2.0f;
    // Floor probe also prevents launching from a stale contact in mid-air.
    glm::vec3 ground=l.pos;
    if(!FDropOnGround(&ground,30,-35,false)||std::abs(ground.z-l.pos.z)>30)return false;
    SlaijumpPlan best;float bestScore=-1e20f;
    for(LO* path:PatrolPaths(psw))
    {
        const CRV* c=PcrvFromLo(path);if(!c||c->as.size()<2)continue;
        float s=0;glm::vec3 q(0);NearestCrvAll(c,l.pos,&q,nullptr,nullptr,&s);
        if(!std::isfinite(s)||glm::length(glm::vec2(q-l.pos))>350)continue;
        for(float off:{0.0f,85.0f,-85.0f,180.0f,-180.0f,300.0f,-300.0f})
        for(int dir:{1,-1})
        {
            if(path==l.path&&(off*l.dir<65||dir!=l.dir))continue;
            SlaijumpPlan plan;
            if(!PlanSlaiJump(l,path,s+off,dir,plan))continue;
            const bool recent=std::find(l.recentPaths.begin(),l.recentPaths.end(),path)!=l.recentPaths.end();
            float score=(path!=l.path?140.0f:0.0f)-(recent?100.0f:0.0f)-0.3f*Distance(l.pos,plan.target);
            if(interest)score+=(away?1.0f:-1.0f)*(Distance(plan.target,*interest)-Distance(l.pos,*interest));
            if(interest&&away&&Distance(plan.target,*interest)<Distance(l.pos,*interest)-40)continue;
            if(score>bestScore){best=plan;bestScore=score;}
        }
    }
    // Look for a safe landing beyond an obstacle even when no patrol curve exists there.
    glm::vec2 forward(std::cos(l.heading),std::sin(l.heading));
    if(interest){glm::vec2 v(*interest-l.pos);if(away)v=-v;if(glm::length(v)>1)forward=glm::normalize(v);}
    for(float angle:{0.0f,0.40f,-0.40f,0.85f,-0.85f,1.57f,-1.57f})
    for(float length:{150.0f,235.0f,320.0f})
    {
        glm::vec2 v(forward.x*std::cos(angle)-forward.y*std::sin(angle),forward.x*std::sin(angle)+forward.y*std::cos(angle));
        glm::vec3 q=l.pos+glm::vec3(v*length,0);
        if(!FDropOnGround(&q,110,-550,false))continue;
        if(now<l.blockedUntil&&Distance(q,l.blockedPos)<100)continue;
        if(interest&&away&&Distance(q,*interest)<Distance(l.pos,*interest)+20)continue;
        SlaijumpPlan plan;if(!PlanSlaiLanding(l,q,nullptr,0,1,plan))continue;
        float score=glm::dot(v,forward)*length*0.5f-std::abs(angle)*30-0.3f*length;
        if(interest)score+=(away?1.0f:-1.0f)*(Distance(q,*interest)-Distance(l.pos,*interest));
        if(score>bestScore){bestScore=score;best=plan;}
    }
    if(best.flight<=0)return false;
    l.jumping=true;l.jumpStarted=now;l.jumpDeadline=now+best.flight+0.8f;
    l.jumpPath=best.path;l.jumpGoal=best.target;l.jumpS=best.s;l.jumpDir=best.dir;
    l.heading=std::atan2(best.velocity.y,best.velocity.x);
    SetStepPhysHeading(st->palo,l.heading);
    st->nMotion=-1;st->vecVel=glm::vec3(0);st->pxpFloor=nullptr;
    SetAloVelocityTo(st->palo,best.velocity);WakeAlo(st->palo);
    RestartClip(l,l.jumpClip);
    std::printf("Slai: jump to path %d, height %.0f, flight %.2f\n",best.path?best.path->oid:-1,best.target.z-l.pos.z,best.flight);
    return true;
}
// Use only discovered upward PAD bindings with a verified landing route. Do not fire mission scripts.
// Launch through the same native STEP velocity/gravity integration as ordinary Slai jumps.
bool TrySlaiPad(SW* psw,LIVE& l,const glm::vec3* interest,bool away,float dt)
{
    STEP* st=PstepFromLo(l.plo); const float now=g_clock.t;
    if(!st||!st->pxpFloor||!l.jumpClip||now<l.nextPad)return false;
    LO* bestPath=nullptr;glm::vec3 bestFrom(0),bestTo(0),bestVel(0);float bestS=0,bestT=0,bestScore=-1e20f;
    const SOST* physics=Psost(st->palo);if(!physics||physics->dvGravity.z>=-1)return false;
    const float g=-physics->dvGravity.z;
    for(OID oid:l.reportedPads)
    {
        LO* pad=PloFindSwObjectByOid(psw,oid);
        if(!pad||!pad->fInWorld||pad->fRemoved||pad->Cid()!=0x53||!pad->pframe)continue;
        CRef* ref=pad->pframe->PrefFindBinding(1088,1);if(!ref||ref->m_tagk!=TAGK_Vector)continue;
        glm::vec3 from=PosAloCenter(static_cast<ALO*>(pad)),to=ref->GetVector();
        if(!FDropOnGround(&from,100,-150,false)||Distance(from,l.pos)>350||!GroundConnection(l.pos,from))continue;
        if(!FDropOnGround(&to,100,-140,false)||SlaiWetPoint(to)||to.z<from.z+65||to.z>from.z+1800)continue;
        float ns=0;LO* path=FindPatrol(psw,to,ns,nullptr,250);
        glm::vec3 q(0);if(!path||path==l.path||!SamplePath(PcrvFromLo(path),ns,q)||!GroundConnection(to,q))continue;
        if(interest&&away&&Distance(to,*interest)<Distance(l.pos,*interest))continue;
        const glm::vec3 delta=to-from;
        const float vz=std::sqrt(2*g*(delta.z+100));
        const float t=(vz+std::sqrt(2*g*100))/g;
        const glm::vec3 vel(delta.x/t,delta.y/t,vz);
        if(!std::isfinite(t)||t<0.2f||t>3.0f||glm::length(glm::vec2(vel))>1000)continue;
        glm::vec3 last=from;bool clear=true;
        for(int i=1;i<=24&&clear;++i){float u=t*i/24;auto at=from+vel*u-glm::vec3(0,0,0.5f*g*u*u);for(float z:{25.0f,100.0f})if(!SlaiClearLine(last+glm::vec3(0,0,z),at+glm::vec3(0,0,z)))clear=false;last=at;}
        if(!clear)continue;
        const float score=-Distance(l.pos,from)+(interest?(away?1:-1)*(Distance(to,*interest)-Distance(l.pos,*interest)):100);
        if(score>bestScore){bestScore=score;bestPath=path;bestFrom=from;bestTo=to;bestVel=vel;bestS=ns;bestT=t;}
    }
    if(!bestPath){l.nextPad=now+2;return false;}
    l.state="pad";
    if(Distance(l.pos,bestFrom)>25){SlaiDrive(l,bestFrom,c_sSlaiWalk,dt);PlayClip(l,l.walk);return true;}
    l.jumping=true;l.jumpStarted=now;l.jumpDeadline=now+bestT+1;l.jumpPath=bestPath;l.jumpGoal=bestTo;l.jumpS=bestS;l.jumpDir=1;l.nextPad=now+15;
    l.heading=std::atan2(bestVel.y,bestVel.x);SetStepPhysHeading(st->palo,l.heading);
    st->nMotion=-1;st->vecVel=glm::vec3(0);st->pxpFloor=nullptr;
    SetAloVelocityTo(st->palo,bestVel);WakeAlo(st->palo);RestartClip(l,l.jumpClip);
    std::printf("Slai: PAD launch toward path %d\n",bestPath->oid);return true;
}
// Check a ballistic arc with body-width clearance, without moving the actor.
bool SlaiRoofArc(const glm::vec3& start,const glm::vec3& vel,float g,float t)
{
    if(!std::isfinite(t)||t<=0||t>2.5f)return false;
    const glm::vec2 v(vel);const float speed=glm::length(v);
    const glm::vec2 side=speed>1?glm::vec2(-v.y,v.x)/speed:glm::vec2(1,0);
    glm::vec3 last=start;
    const int n=std::clamp(int(std::ceil(t/0.06f)),8,44);
    for(int i=1;i<=n;++i){
        const float u=t*float(i)/n;
        const glm::vec3 q=start+vel*u-glm::vec3(0,0,0.5f*g*u*u);
        for(float z:{18.f,75.f,145.f}){
            if(!SlaiClearLine(last+glm::vec3(0,0,z),q+glm::vec3(0,0,z)))return false;
            if(z==75.f)for(float width:{-24.f,24.f}){
                const glm::vec3 off(side*width,z);
                if(!SlaiClearLine(last+off,q+off))return false;
            }
        }
        last=q;
    }
    return true;
}
bool SlaiRoofFloor(const glm::vec3& q)
{
    if(SlaiWetPoint(q))return false;
    for(glm::vec2 v:{glm::vec2(0),glm::vec2(48,0),glm::vec2(-48,0),glm::vec2(0,48),glm::vec2(0,-48)}){
        glm::vec3 p=q+glm::vec3(v,0);
        if(!FDropOnGround(&p,35,-35,false)||std::abs(p.z-q.z)>25||SlaiWetPoint(p))return false;
        if(!SlaiClearLine(p+glm::vec3(0,0,12),p+glm::vec3(0,0,155)))return false;
    }
    return true;
}
bool SlaiRoofSecond(const glm::vec3& start,const glm::vec3& target,float g,glm::vec3& vel,float& flight)
{
    if(!std::isfinite(g)||g<=1)return false;
    const glm::vec3 d=target-start;
    const float rise=std::max(0.f,d.z)+105.f;
    if(rise>420||d.z<-500)return false;
    const float vz=std::sqrt(2*g*rise);
    flight=(vz+std::sqrt(std::max(0.f,vz*vz-2*g*d.z)))/g;
    if(!std::isfinite(flight)||flight<0.2f||flight>2.0f||glm::length(glm::vec2(d))/flight>c_sSlaiRun)return false;
    vel=glm::vec3(d.x/flight,d.y/flight,vz);
    return SlaiRoofArc(start,vel,g,flight);
}
bool TrySlaiRoof(LIVE& l,const glm::vec3* interest,bool away)
{
    const float now=g_clock.t;
    STEP* st=PstepFromLo(l.plo);
    if(!st||!st->pxpFloor||!l.jumpClip||!l.doubleJumpClip||now<l.nextRoof||now<l.nextJump)return false;
    l.nextRoof=now+RandomRange(5,9);
    glm::vec3 floor=l.pos;
    if(!FDropOnGround(&floor,30,-35,false)||std::abs(floor.z-l.pos.z)>30||SlaiWetPoint(floor))return false;
    const SOST* physics=Psost(st->palo);
    if(!physics||!std::isfinite(physics->dvGravity.z)||physics->dvGravity.z>=-1)return false;
    const float g=-physics->dvGravity.z;
    // First jump is almost vertical. Shift toward the roof only in the second phase,
    // so an adjacent facade does not block the ascending body.
    const float rise=340.f, vz=std::sqrt(2*g*rise), t=vz/g;
    const glm::vec3 launch(0,0,vz),apex=l.pos+glm::vec3(0,0,rise);
    if(!SlaiRoofArc(l.pos,launch,g,t))return false;
    glm::vec2 forward(std::cos(l.heading),std::sin(l.heading));
    if(interest){glm::vec2 v(*interest-l.pos);if(away)v=-v;if(glm::length(v)>1)forward=glm::normalize(v);}
    glm::vec3 best(0);float bestScore=-1e20f,bestT=0;
    for(float angle:{0.f,0.7854f,-0.7854f,1.5708f,-1.5708f,2.3562f,-2.3562f,3.14159f})
    for(float r:{175.f,285.f,400.f}){
        glm::vec2 v(forward.x*std::cos(angle)-forward.y*std::sin(angle),forward.x*std::sin(angle)+forward.y*std::cos(angle));
        glm::vec3 q=l.pos+glm::vec3(v*r,0);
        // Highest static surface within the permitted vertical band; no required path.
        if(!FDropOnGround(&q,610,140,false)||q.z-l.pos.z<145||q.z-l.pos.z>600)continue;
        if(now<l.blockedUntil&&Distance(q,l.blockedPos)<100)continue;
        if(interest&&!away&&Distance(q,*interest)>Distance(l.pos,*interest)-50)continue;
        if(interest&&away&&Distance(q,*interest)<Distance(l.pos,*interest)-60)continue;
        if(!SlaiRoofFloor(q))continue;
        glm::vec3 vel(0);float flight=0;
        if(!SlaiRoofSecond(apex,q,g,vel,flight))continue;
        float score=0.35f*(q.z-l.pos.z)-0.25f*r-25*std::abs(angle);
        if(interest)score+=(away?1.f:-1.f)*(Distance(q,*interest)-Distance(l.pos,*interest));
        if(score>bestScore){bestScore=score;best=q;bestT=flight;}
    }
    if(bestT<=0)return false;
    l.roofFlight=true;l.roofSecondPending=true;l.roofSecondAt=now+t;l.roofApex=apex;
    l.jumping=true;l.jumpStarted=now;l.jumpDeadline=now+t+bestT+1.f;
    l.jumpGoal=best;l.jumpPath=nullptr;l.nextRoof=now+RandomRange(20,35);
    l.heading=std::atan2(best.y-l.pos.y,best.x-l.pos.x);SetStepPhysHeading(st->palo,l.heading);
    st->nMotion=-1;st->vecVel=glm::vec3(0);st->pxpFloor=nullptr;
    SetAloVelocityTo(st->palo,launch);WakeAlo(st->palo);RestartClip(l,l.jumpClip);
    l.state="roof-jump";
    std::printf("Slai: roof first jump to %.0f %.0f %.0f\n",best.x,best.y,best.z);
    return true;
}
bool StepSlaiJump(SW* psw,LIVE& l,STEP* st,float now)
{
    if(!l.jumping)return false;
    const float vz=VelAloPos(st->palo).z;
    if(l.roofSecondPending && now>=l.roofSecondAt){
        l.roofSecondPending=false;
        const SOST* physics=Psost(st->palo);
        glm::vec3 vel(0);float t=0;
        // Replan from the actual feet. Cancel safely if the first jump hit a ceiling,
        // was interrupted by damage, or the target has ceased to be reachable.
        if(!st->pxpFloor&&now>=l.hurtUntil&&Distance(l.pos,l.roofApex)<95&&physics&&
            SlaiRoofFloor(l.jumpGoal)&&SlaiRoofSecond(l.pos,l.jumpGoal,-physics->dvGravity.z,vel,t)){
            st->nMotion=-1;st->vecVel=glm::vec3(0);st->pxpFloor=nullptr;
            SetAloVelocityTo(st->palo,vel);WakeAlo(st->palo);RestartClip(l,l.doubleJumpClip);
            l.jumpDeadline=now+t+0.8f;
            std::printf("Slai: roof second jump\n");return true;
        }
        l.jumpDeadline=now+1.5f;
        std::printf("Slai: roof second jump cancelled\n");
    }
    glm::vec3 q=l.pos;
    const bool landed=!l.roofSecondPending&&now-l.jumpStarted>0.18f&&vz<100&&st->pxpFloor&&FDropOnGround(&q,30,-35,false)&&std::abs(q.z-l.pos.z)<30&&!SlaiWetPoint(q);
    if(!landed&&now<l.jumpDeadline)
    {
        st->nMotion=-1;st->vecVel=glm::vec3(0);
        return true; // do not reapply launch velocity, hold, or snap to path
    }
    l.jumping=false;l.roofSecondPending=false;l.roofFlight=false;st->nMotion=0;st->vecVel=glm::vec3(0);
    l.lastProgress=now;l.nextProgressCheck=now+0.5f;l.progressPos=l.pos;l.nextJump=now+3;
    if(landed)
    {
        const bool arrived=Distance(l.pos,l.jumpGoal)<110;
        if(arrived&&l.jumpPath){RememberPath(l,l.path);l.path=l.jumpPath;l.s=l.jumpS;l.dir=l.jumpDir;}
        else {float s=0;LO* path=FindPatrol(psw,l.pos,s,nullptr,150);if(path){RememberPath(l,l.path);l.path=path;l.s=s;}else l.path=nullptr;}
        l.routeSince=now; // last dry point is updated only by the stable-floor sampler
        std::printf("Slai: jump landed (%s)\n",arrived?"target":"other floor");
    }
    else {l.path=nullptr;l.nextSearch=now;std::printf("Slai: jump timeout\n");}
    l.jumpPath=nullptr;
    return false;
}
// Water is not a grounded route. First attempt a native ballistic escape toward a dry ledge.
// A last-dry-position recovery is a bounded fallback, never an every-frame teleport.
void ResetSlaiNav(LIVE& l){l.navNodes.clear();l.navRoute.clear();l.navCursor=0;}
bool RecoverSlaiWater(SW* psw,LIVE& l,STEP* st,float now,float dt)
{
    ACTOR* actor=PactorFromLo(l.plo);
    bool wet=SlaiWetPoint(l.pos);
    // A water-exit message may lag a teleport or touchdown; corroborate the old contact.
    if(!wet && actor && actor->ploWater && actor->ploWater->fInWorld){
        if(OXA* ox=Poxa(static_cast<ALO*>(actor->ploWater)))
            wet=l.pos.x>=ox->posMin.x&&l.pos.x<=ox->posMax.x&&l.pos.y>=ox->posMin.y&&l.pos.y<=ox->posMax.y&&l.pos.z<ox->posMax.z-5&&l.pos.z+75>ox->posMin.z;
    }
    if(l.waterJump){
        if(StepSlaiJump(psw,l,st,now)){l.state="water-jump";return true;}
        l.waterJump=false;
        if(!wet){l.waterSince=0;return false;}
    }
    if(!wet){l.waterSince=0;return false;}
    if(l.waterSince==0){l.waterSince=now;l.jumping=false;l.roofSecondPending=false;l.roofFlight=false;l.jumpPath=nullptr;ResetSlaiNav(l);l.path=nullptr;l.freeUntil=0;}
    if(now>=l.nextWaterTry && now-l.waterSince<2.2f){
        l.nextWaterTry=now+1.0f;
        SlaijumpPlan best;float score=1e20f;
        auto test=[&](glm::vec3 q){
            if(!FDropOnGround(&q,130,-85,false)||SlaiWetPoint(q))return;
            SlaijumpPlan plan;if(!PlanSlaiLanding(l,q,nullptr,0,1,plan))return;
            const float d=Distance(l.pos,q);if(d<score){score=d;best=plan;}
        };
        if(l.hasSafePos)test(l.safePos);
        for(float r:{110.f,200.f,300.f})for(int i=0;i<8;++i){const float a=i*0.78539816f;test(l.pos+glm::vec3(std::cos(a)*r,std::sin(a)*r,0));}
        if(best.flight>0){
            l.waterJump=true;l.jumping=true;l.jumpStarted=now;l.jumpDeadline=now+best.flight+0.8f;l.jumpGoal=best.target;
            l.jumpPath=nullptr;l.heading=std::atan2(best.velocity.y,best.velocity.x);SetStepPhysHeading(st->palo,l.heading);
            st->nMotion=-1;st->vecVel=glm::vec3(0);st->pxpFloor=nullptr;SetAloVelocityTo(st->palo,best.velocity);WakeAlo(st->palo);RestartClip(l,l.jumpClip);
            l.state="water-jump";std::printf("Slai: water escape jump\n");return true;
        }
    }
    if(now-l.waterSince>=2.2f){
        glm::vec3 safe=l.hasSafePos?l.safePos:l.spawn;
        if(!FDropOnGround(&safe,45,-60,false)||SlaiWetPoint(safe))safe=l.spawn;
        if(FDropOnGround(&safe,45,-60,false)&&!SlaiWetPoint(safe)){
            l.blockedPos=l.pos;l.blockedUntil=now+25;l.pos=safe;l.heading+=3.14159265f;PlaceAt(l.plo,safe,l.heading);
            l.path=nullptr;l.nextSearch=now+2;l.freeUntil=0;l.nextLocal=now;l.nextWaterTry=now+2;l.waterSince=0;
            l.lastProgress=now;l.progressPos=safe;l.nextProgressCheck=now+0.5f;PlaySlai(l,false);l.state="water-recover";
            std::printf("Slai: water recovery to dry ground\n");return true;
        }
    }
    l.state="water-recover";SlaiDrive(l,l.pos,0,dt);return true;
}
// A small private navigation graph: allow a detour to initially run toward the player,
// rather than insisting that every single step increases escape distance. No global navmesh yet.
bool StepSlaiDetour(LIVE& l,const glm::vec3& threat,bool stuck,float now,float dt)
{
    if(!l.navRoute.empty()){
        if(stuck&&now-l.navStarted>1.3f){ResetSlaiNav(l);l.navCooldown=now+0.7f;return false;}
        while(!l.navRoute.empty()&&Distance(l.pos,l.navRoute.front())<45)l.navRoute.pop_front();
        if(l.navRoute.empty()){ResetSlaiNav(l);l.navCooldown=now+0.7f;return false;}
        if(!GroundConnection(l.pos,l.navRoute.front())){ResetSlaiNav(l);l.navCooldown=now+0.7f;return false;}
        SlaiDrive(l,l.navRoute.front(),c_sSlaiRun,dt);PlayClip(l,l.run?l.run:l.walk);l.state="flee-detour";return true;
    }
    if(l.navNodes.empty()){
        if(!stuck||now<l.navCooldown)return false;
        l.navNodes.push_back({l.pos,-1});l.navCursor=0;l.navStarted=now;l.navNext=now;
    }
    if(now>=l.navNext){
        l.navNext=now+0.07f;
        // Two nodes per slice; keep the physics queries bounded to avoid a large frame spike.
        for(int batch=0;batch<2 && l.navCursor<l.navNodes.size()&&l.navNodes.size()<56;++batch){
            const int idx=int(l.navCursor++);const glm::vec3 from=l.navNodes[idx].pos;
            for(int i=0;i<8&&l.navNodes.size()<56;++i){
                const float angle=i*0.78539816f;glm::vec3 q=from+glm::vec3(140*std::cos(angle),140*std::sin(angle),0);
                if(Distance(q,l.navNodes[0].pos)>800||!FDropOnGround(&q,50,-65,false)||SlaiWetPoint(q))continue;
                bool duplicate=false;for(const auto& n:l.navNodes)if(Distance(n.pos,q)<80){duplicate=true;break;}
                if(duplicate||!GroundConnection(from,q))continue;
                l.navNodes.push_back({q,idx});
            }
        }
        int best=-1;float value=0;const float base=Distance(l.navNodes[0].pos,threat);
        for(size_t i=1;i<l.navNodes.size();++i){const auto& q=l.navNodes[i].pos;const float gain=Distance(q,threat)-base;const float score=gain+0.12f*Distance(q,l.navNodes[0].pos);if(score>value){best=int(i);value=score;}}
        if((best>0&&value>300)||l.navCursor>=l.navNodes.size()||l.navNodes.size()>=56||now-l.navStarted>1.5f){
            if(best<0||value<55){ResetSlaiNav(l);l.navCooldown=now+2;return false;}
            for(int i=best;i>0;i=l.navNodes[i].parent)l.navRoute.push_front(l.navNodes[i].pos);
            l.navNodes.clear();l.navStarted=now;l.lastProgress=now;l.progressPos=l.pos;l.nextProgressCheck=now+0.5f;
            return StepSlaiDetour(l,threat,false,now,dt);
        }
    }
    SlaiDrive(l,l.pos,0,dt);PlaySlai(l,false);l.state="flee-plan";return true;
}
bool LiveGuard(LO* o)
{
    if (!o || !o->fInWorld || o->fRemoved || !FCidDerivedFrom(o->Cid(),0x38)) return false;
    // Only ordinary guards, not boss/special subclasses. Positive native HP required.
    GUARD* g = PguardFromLo(o);
    return g && FModSlaiOrdinaryGuard(o) && g->n33C > 0 && !FGuardState(g,42);
}
LO* NearbyGuard(const glm::vec3& pos)
{
    LO* best = nullptr; float dist = 1500.0f;
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
    CTMSG m; m.ploSender=l.plo; m.ploReceiver=target; m.grfFilter=4; m.nState=-10;
    m.pos=q; m.vec=glm::vec3(std::cos(l.heading),std::sin(l.heading),0.0f)*150.0f;
    PaloAcceptContact(&m);
}

bool StartSlaiPath(SW* psw, LIVE& l)
{
    if (!l.walk) return false;
    const bool automatic = l.pg->patrol < 0;
    LO* player = PloPoCurrent(psw);
    if (automatic && (!player || player == l.plo || !FCidDerivedFrom(player->Cid(), 0x30))) return false;
    glm::vec3 anchor = l.hasSpawn ? l.pos : automatic ? PosAloCenter(static_cast<ALO*>(player)) :
        glm::vec3(l.pg->at[0], l.pg->at[1], l.pg->at[2]);
    float s = 0.0f;
    LO* path = automatic ? FindPatrol(psw, anchor, s, nullptr, l.hasSpawn ? 150.0f : std::numeric_limits<float>::max()) : PloFindSwObjectByOid(psw, l.pg->patrol);
    const CRV* c = PcrvFromLo(path);
    if (!c || c->as.size() < 2 || !(c->as.back() - c->as.front() > 80.0f)) return false;
    if (!automatic) NearestCrvAll(c, anchor, nullptr, nullptr, nullptr, &s);
    if (!std::isfinite(s)) return false;
    l.path = path;
    l.s = std::clamp(s, c->as.front(), c->as.back());
    l.dir = l.s >= c->as.back() - 1.0f ? -1 : 1;
    // Avoid spawning directly on the player on the automatic path.
    if (automatic && !l.hasSpawn)
    {
        float trial = l.s + float(l.dir) * 300.0f;
        if (trial > c->as.back() || trial < c->as.front()) trial = l.s - float(l.dir) * 300.0f;
        trial=std::clamp(trial,c->as.front(),c->as.back());
        glm::vec3 q(0), tangent(0); EvalCrvS(c,trial,&q,&tangent);
        if (FDropOnGround(&q,100,-250,false)&&!SlaiWetPoint(q)) l.s=trial;
    }
    std::printf("Slai: path %d, start %.1f, length %.1f, walk %d\n", path->oid, l.s, c->as.back() - c->as.front(), l.walk->oid);
    return true;
}

// Pickups override routine activities. A reachable cure wins over coins,
// including between attacks; never interrupt a swing, escape or airborne recovery.
bool StepSlaiPickup(LIVE& l,float now,float dt,bool healOnly)
{
    if(l.fleeing || l.jumping || l.hp<=0 || now<l.hurtUntil || now<l.attackUntil){l.pickupUntil=0;l.pickupStarted=0;return false;}
    if(l.pickupHealOnly!=healOnly){l.pickupHealOnly=healOnly;l.nextPickupSearch=now;}
    if(l.pickupUntil>now && ((healOnly&&!l.pickupHeal)||(l.pickupHeal?l.hp>=c_nSlaiMaxHP:l.coins>=c_nSlaiMaxCoins))){
        l.pickupUntil=0;l.pickupStarted=0;l.nextPickupSearch=now;
    }
    if(healOnly&&l.hp>=c_nSlaiMaxHP)return false;
    if(l.pickupStarted>0 && now-l.pickupStarted>8.0f){
        l.pickupSkipPos=l.pickupGoal;l.pickupSkipUntil=now+5;l.pickupUntil=0;l.pickupStarted=0;
    }
    if(now>=l.nextPickupSearch){
        l.nextPickupSearch=now+0.45f;
        l.pickupUntil=0;
        auto candidates=ASlaiPickupTargets(l.pos+glm::vec3(0,0,65),2000.0f);
        std::sort(candidates.begin(),candidates.end(),[&](const auto& a,const auto& b){
            if(a.heal!=b.heal)return a.heal>b.heal;
            return Distance(a.pos,l.pos)<Distance(b.pos,l.pos);
        });
        for(const auto& c:candidates){
            if(c.heal ? l.hp>=c_nSlaiMaxHP : (healOnly||l.coins>=c_nSlaiMaxCoins))continue;
            if(std::abs(c.pos.z-l.pos.z)>130||!SlaiClearLine(l.pos+glm::vec3(0,0,65),c.pos))continue;
            glm::vec3 q=c.pos;
            if(!FDropOnGround(&q,40,-150,false)||SlaiWetPoint(q)||std::abs(q.z-l.pos.z)>65||!GroundConnection(l.pos,q))continue;
            if(now<l.pickupSkipUntil&&Distance(q,l.pickupSkipPos)<100)continue;
            if(l.pickupStarted==0||l.pickupHeal!=c.heal||Distance(q,l.pickupGoal)>100)l.pickupStarted=now;
            l.pickupHeal=c.heal;l.pickupGoal=q;l.pickupUntil=now+0.55f;break;
        }
    }
    if(now>=l.pickupUntil){l.pickupStarted=0;return false;}
    if(!GroundConnection(l.pos,l.pickupGoal)){l.pickupUntil=0;l.pickupStarted=0;return false;}
    l.state=l.pickupHeal?"heal":"pickup";
    l.restUntil=0;l.lastProgress=now;l.progressPos=l.pos;
    const float distance=glm::length(glm::vec2(l.pickupGoal-l.pos));
    if(distance>10){SlaiDrive(l,l.pickupGoal,std::min(c_sSlaiWalk,distance*4.0f),dt);PlayClip(l,l.walk);}
    else {SlaiDrive(l,l.pos,0,dt);PlaySlai(l,false);}
    return true;
}

void StepSlai(SW* psw,LIVE& l)
{
    if(!l.plo->fInWorld||l.plo->fRemoved||l.plo==PloPoCurrent(psw))return;
    const float now=g_clock.t,dt=std::clamp(g_clock.dt,0.0f,0.1f);
    if(FGamePausedForAi()||FSlaiSafeHouseOpen()||dt<=0)return;
    STEP* step=PstepFromLo(l.plo);if(!step)return;
    ALO* a=step->palo;
    l.pos=SlaiFeet(l.plo);
    auto hold=[&]{SlaiDrive(l,l.pos,0,dt);};
    if(l.hp<=0)
    {
        l.state="defeated";
        if(l.respawnAt==0){l.respawnAt=now+6;RestartClip(l,l.defeat);}
        if(now<l.respawnAt){hold();return;}
        std::printf("Slai: respawn, HP 0 -> %d\n",c_nSlaiMaxHP);
        l.hp=c_nSlaiMaxHP;l.coins=0;l.pickupUntil=0;l.pickupStarted=0;l.pickupSkipUntil=0;l.nextPickupSearch=now;l.respawnAt=0;l.hurtUntil=0;l.nextHit=now+2;
        l.attackUntil=0;l.attackTarget=nullptr;l.nextCombat=0;l.fleeing=false;l.restUntil=0;
        ResetSocial(l,now,false);l.socialPlayer=nullptr;
        l.jumping=false;l.roofSecondPending=false;l.roofFlight=false;l.nextRoof=now+10;l.jumpPath=nullptr;l.nextJump=now+3;l.waterJump=false;l.waterSince=0;ResetSlaiNav(l);
        l.path=l.spawnPath;l.s=l.spawnS;l.pos=l.spawn;l.heading=l.spawnHeading;
        l.recentPaths.clear();l.nextRecovery=0;l.blockedUntil=0;l.freeUntil=0;l.nextLocal=now;l.nextPad=now+3;l.routeSince=now;l.lastProgress=now;l.nextProgressCheck=now+0.5f;l.progressPos=l.spawn;
        l.nextRest=now+SlaiExploreTime();l.nextSearch=now;l.nextSwitch=now+3;
        PlaceAt(l.plo,l.spawn,l.heading);PlaySlai(l,false);hold();return;
    }
    if(!l.hasSpawn)
    {
        if(now<l.nextSearch){hold();return;}l.nextSearch=now+1;
        if(!StartSlaiPath(psw,l)){hold();PlaySlai(l,false);return;}
        glm::vec3 q(0);if(!SamplePath(PcrvFromLo(l.path),l.s,q)){l.path=nullptr;hold();return;}
        l.hasSpawn=true;l.spawn=q;l.spawnS=l.s;l.spawnPath=l.path;l.spawnHeading=l.heading;
        l.pos=q;l.safePos=q;l.hasSafePos=true;l.progressPos=q;l.lastProgress=now;l.nextProgressCheck=now+0.5f;
        l.routeSince=now;l.nextRest=now+SlaiExploreTime();l.nextSwitch=now+3;
        ResetSocial(l,now,false);
        PlaceAt(l.plo,q,l.heading);hold();return;
    }
    if(RecoverSlaiWater(psw,l,step,now,dt))return;
    if(StepSlaiJump(psw,l,step,now)){l.state=l.roofFlight?(l.roofSecondPending?"roof-jump":"roof-double"):"jump";return;}
    // Save a dry, stable location, not a water surface or submerged floor.
    if(step->pxpFloor && now>=l.nextDrySample){
        l.nextDrySample=now+0.35f;glm::vec3 q=l.pos;
        if(FDropOnGround(&q,35,-40,false)&&std::abs(q.z-l.pos.z)<30&&!SlaiWetPoint(q)){
            bool dry=true;for(glm::vec2 off:{glm::vec2(45,0),glm::vec2(-45,0),glm::vec2(0,45),glm::vec2(0,-45)}){
                glm::vec3 probe=q+glm::vec3(off,0);if(!FDropOnGround(&probe,35,-45,false)||SlaiWetPoint(probe)){dry=false;break;}}
            if(dry){l.safePos=q;l.hasSafePos=true;}
        }
    }
    if(l.hasSafePos && !step->pxpFloor && l.pos.z<l.safePos.z-350)
    {
        l.pos=l.safePos;PlaceAt(l.plo,l.pos,l.heading);l.path=nullptr;l.nextSearch=now;l.lastProgress=now;hold();return;
    }
    if(now<l.hurtUntil){l.state="hurt";hold();return;}
    ScanSlaiPads(psw,l);
    if(!l.path && now>=l.nextSearch){l.nextSearch=now+0.5f;if(StartSlaiPath(psw,l)){l.routeSince=now;}}
    const CRV* c=PcrvFromLo(l.path);
    if(!c||c->as.size()<2){l.path=nullptr;c=nullptr;}
    // Resynchronize route progress to actual physical position, not an assumed kinematic distance.
    if(c){float nearS=l.s;glm::vec3 raw(0);NearestCrvAll(c,l.pos,&raw,nullptr,nullptr,&nearS);
    if(std::isfinite(nearS))l.s=BoundS(c,nearS);}
    LO* player=PloPoCurrent(psw);
    const bool validPlayer=player&&player!=l.plo&&player->fInWorld&&!player->fRemoved&&FCidDerivedFrom(player->Cid(),0x34);
    if(l.socialPlayer!=(validPlayer?player:nullptr))
    {
        ResetSocial(l,now,false);l.socialPlayer=validPlayer?player:nullptr;
    }
    const bool carm=validPlayer&&FPcCarm(player);
    glm::vec3 pp=validPlayer?PosAloCenter(static_cast<ALO*>(player)):glm::vec3(0);
    const glm::vec3 eye=l.pos+glm::vec3(0,0,65);
    const float d=validPlayer?Distance(pp,eye):1e10f;
    const bool seen=validPlayer&&d<=c_sSlaiDetect&&SlaiClearLine(eye,pp);
    bool discovered=false;
    if(carm&&seen&&d<=c_sSlaiObserve)if(STEP* st=PstepFromLo(player))
    {
        const float rad=RadStepHeading(st);const glm::vec2 f(std::cos(rad),std::sin(rad)),to(eye.x-pp.x,eye.y-pp.y);
        const float t2=glm::dot(to,to);discovered=t2<1||glm::dot(f,to)>=c_gSlaiVisual*std::sqrt(t2);
    }
    if(!carm||d>=c_sSlaiSafe){l.fleeing=false;ResetSlaiNav(l);}
    else if(discovered&&!l.fleeing){l.fleeing=true;l.nextDirection=now;}
    if(l.fleeing){
        if(l.following||l.visiting||l.followHolding)ResetSocial(l,now,true);
        l.attackUntil=0;l.attackTarget=nullptr;l.restUntil=0;
    }
    // Combat persists through attack cooldowns; only an actual approach can translate the actor.
    if(!l.fleeing)
    {
        if(!LiveGuard(l.attackTarget)){if(l.attackTarget)l.nextCombat=now+RandomRange(4,10);l.attackTarget=nullptr;l.attackUntil=0;}
        if(!l.attackTarget&&now>=l.nextCombat&&!l.attacks.empty()){l.attackTarget=NearbyGuard(l.pos);if(l.attackTarget)l.lastGuardSeen=now;}
        if(StepSlaiPickup(l,now,dt,l.attackTarget!=nullptr))return;
        if(l.attackTarget)
        {
            const glm::vec3 target=PosAloCenter(static_cast<ALO*>(l.attackTarget));const float dist=Distance(eye,target);
            const bool clear=SlaiClearLine(eye,target);if(clear)l.lastGuardSeen=now;
            if(dist>2500||now-l.lastGuardSeen>5.0f){l.attackTarget=nullptr;l.attackUntil=0;l.nextCombat=now+RandomRange(4,10);}
            else
            {
                if(l.visiting||l.following||l.followHolding)ResetSocial(l,now,true);
                l.state="combat";l.restUntil=0;l.lastProgress=now;FaceSlai(l,target,dt);
                if(now<l.attackUntil){if(!l.struck&&now>=l.strikeAt){l.struck=true;StrikeSlai(l);}hold();return;}
                const glm::vec3 v=target-l.pos;const float angle=std::abs(std::remainder(std::atan2(v.y,v.x)-RadStepHeading(step),6.2831853f));
                if(clear&&dist<=190&&angle<0.45f&&now>=l.nextAttack&&!l.attacks.empty())
                {
                    ASEG* clip=l.attacks[std::uniform_int_distribution<size_t>(0,l.attacks.size()-1)(s_rng)];
                    const float duration=ClipTime(clip,0.8f);l.struck=false;l.attackUntil=now+duration;l.strikeAt=now+duration*0.45f;l.nextAttack=l.attackUntil+1.5f;
                    RestartClip(l,clip);hold();return;
                }
                bool approach=false;glm::vec3 q=l.pos;
                if(clear&&dist>190)
                {
                    glm::vec2 dir=glm::vec2(target-l.pos);float len=glm::length(dir);
                    if(len>1){dir/=len;q=l.pos+glm::vec3(dir*std::min(100.0f,len-std::min(150.0f,len)),0);approach=GroundConnection(l.pos,q);}
                }
                if(!approach&&dist>190)approach=LocalGoal(l,target,false,false,q);
                if(approach)SlaiDrive(l,q,c_sSlaiWalk,dt);else hold();
                PlayClip(l,approach?(l.combatWalk?l.combatWalk:l.walk):(l.combatIdle?l.combatIdle:l.idle));return;
            }
        }
    }
    l.pickupUntil=0;l.pickupStarted=0;
    // A visit is scheduled once, not rolled every frame. Travel and stay have distinct deadlines.
    if(!l.fleeing&&validPlayer)
    {
        if(l.visiting&&((l.visitEnd>0&&now>=l.visitEnd)||
            (l.visitEnd==0&&now>=l.visitDeadline)||d>c_sSlaiVisitRange))
        {
            ResetSocial(l,now,true);
            l.nextRest=now+SlaiExploreTime();
            std::printf("Slai: visit ended\n");
        }
        if(!l.visiting&&now>=l.nextVisit)
        {
            l.nextVisit=now+RandomRange(60,120);
            if(d<=c_sSlaiVisitRange&&now>=l.socialCooldown&&RandomRange(0,1)<0.65f)
            {
                l.visiting=true;l.followHolding=false;l.holdUntil=0;
                l.visitDeadline=now+30;l.visitEnd=0;l.freeUntil=0;l.nextLocal=now;
                l.lastProgress=now;l.progressPos=l.pos;
                std::printf("Slai: spontaneous visit started\n");
            }
        }
        if(seen)l.lastPlayerSeen=now;
        const bool tracking=seen||(l.following&&d<=c_sSlaiDetect&&now-l.lastPlayerSeen<=2.0f);
        const bool wants=l.visiting||(carm&&tracking&&now>=l.socialCooldown);
        if(wants&&!l.following)
        {
            l.following=true;l.followHolding=false;l.holdUntil=0;l.freeUntil=0;l.nextLocal=now;
            l.lastProgress=now;l.progressPos=l.pos;
        }
        else if(!wants&&l.following)
        {
            l.following=false;l.followHolding=false;l.holdUntil=0;l.freeUntil=0;l.nextLocal=now;
        }
    }
    const bool follow=validPlayer&&!l.fleeing&&l.following;
    const bool interested=follow||l.fleeing;
    float speed=c_sSlaiWalk;bool move=true;
    if(interested)
    {
        if(l.restUntil>0){l.restUntil=0;l.nextRest=now+SlaiExploreTime();}
        if(l.fleeing)speed=l.run?c_sSlaiRun:c_sSlaiWalk;
        else
        {
            const float stop=l.visiting?c_sSlaiVisitStop:c_sSlaiFollowStop;
            const float resume=l.visiting?c_sSlaiVisitResume:c_sSlaiFollowResume;
            // For visits, require sight before declaring arrival; don't wait behind a wall.
            if(!l.followHolding&&d<=stop&&(!l.visiting||seen))
            {
                l.followHolding=true;l.holdUntil=now+2;l.freeUntil=0;l.nextLocal=now;
                if(l.visiting&&l.visitEnd==0)l.visitEnd=now+RandomRange(10,20);
            }
            if(l.followHolding&&now>=l.holdUntil&&d>=resume)
            {
                l.followHolding=false;l.freeUntil=0;l.nextLocal=now;
                l.lastProgress=now;l.progressPos=l.pos;l.nextProgressCheck=now+0.5f;
            }
            if(l.followHolding){move=false;if(seen)FaceSlai(l,pp,dt);}
            else speed=std::clamp((d-stop)*1.4f,90.0f,c_sSlaiWalk);
        }
        glm::vec3 f(0),b(0);bool vf=SamplePath(c,l.s+180,f),vb=SamplePath(c,l.s-180,b);
        if(now>=l.nextDirection)
        {
            if(vf&&vb)
            {
                float df=Distance(f,pp),db=Distance(b,pp);
                if(std::abs(df-db)>70)l.dir=l.fleeing?(df>db?1:-1):(df<db?1:-1);
            }
            else if(vf!=vb)l.dir=vf?1:-1;
            l.nextDirection=now+1.5f;
        }
    }
    else
    {
        if(now>=l.nextRest&&now>=l.restUntil){l.restUntil=now+SlaiRestTime();l.nextRest=l.restUntil+SlaiExploreTime();l.restClip=l.idles.empty()?l.idle:l.idles[std::uniform_int_distribution<size_t>(0,l.idles.size()-1)(s_rng)];}
        if(now<l.restUntil){l.state="rest";hold();PlayClip(l,l.restClip);l.lastProgress=now;return;}
    }
    if(!move){l.state=l.visiting?"visit-rest":"observe";hold();l.actualSpeed=0;PlaySlai(l,false);l.lastProgress=now;l.progressPos=l.pos;return;}
    if(now>=l.nextProgressCheck)
    {
        if(glm::length(glm::vec2(l.pos-l.progressPos))>25)l.lastProgress=now;
        l.progressPos=l.pos;l.nextProgressCheck=now+0.5f;
    }
    const bool stuck=now-l.lastProgress>0.85f, old=now-l.routeSince>30.0f;
    l.state=l.fleeing?"flee":l.visiting?"visit":follow?"follow":"explore";
    // Explore upward occasionally, seek high ground during a blocked escape, or
    // reach an elevated player. Never interrupt rest, combat, or an active jump.
    if((stuck||(follow&&pp.z-l.pos.z>160)||(!interested&&now-l.routeSince>15))&&
        l.navNodes.empty()&&l.navRoute.empty()&&TrySlaiRoof(l,interested?&pp:nullptr,l.fleeing))return;
    if(stuck && now>=l.nextRecovery && l.navNodes.empty() && l.navRoute.empty()){
        l.blockedPos=l.pos;l.blockedUntil=now+18;l.freeUntil=0;l.nextLocal=0;l.nextRecovery=now+1.0f;
        if(TrySlaiPad(psw,l,interested?&pp:nullptr,l.fleeing,dt))return;
        if(TrySlaiJump(psw,l,interested?&pp:nullptr,l.fleeing)){l.state="jump";return;}
    }
    if(l.fleeing&&StepSlaiDetour(l,pp,stuck,now,dt))return;
    // Follow/flee in the world, not merely along a path. Keep waypoints long enough to make progress.
    const bool freeMove=interested||!c||stuck||now<l.freeUntil;
    if(freeMove)
    {
        if((interested||!c||stuck)&&(now>=l.nextLocal||Distance(l.pos,l.freeGoal)<40||!GroundConnection(l.pos,l.freeGoal)))
        {
            l.nextLocal=now+0.6f;
            if(LocalGoal(l,pp,l.fleeing,!interested,l.freeGoal))l.freeUntil=now+1.5f;
            else l.freeUntil=0;
        }
        if(now<l.freeUntil&&GroundConnection(l.pos,l.freeGoal))
        {
            SlaiDrive(l,l.freeGoal,speed,dt);
            const bool run=l.fleeing&&l.run;
            const float actual=glm::length(glm::vec2(VelAloPos(a)));
            l.actualSpeed=glm::mix(l.actualSpeed,actual,std::min(dt*8,1.0f));
            PlayClip(l,run?l.run:l.walk,true,std::clamp(l.actualSpeed/(run?c_sSlaiRun:c_sSlaiWalk),0.25f,1.15f));
            return;
        }
        if(TrySlaiPad(psw,l,interested?&pp:nullptr,l.fleeing,dt))return;
        if(TrySlaiJump(psw,l,interested?&pp:nullptr,l.fleeing))return;
    }
    if((old||!c)&&TrySlaiPad(psw,l,nullptr,false,dt))return;
    if(now>=l.nextSwitch)
    {
        l.nextSwitch=now+0.5f;
        if((now-l.routeSince>3||stuck||l.fleeing)&&SwitchPath(psw,l,l.fleeing?&pp:nullptr,old||stuck))
        {c=PcrvFromLo(l.path);l.nextSwitch=now+3;}
    }
    if(!c){hold();PlaySlai(l,false);return;}
    const float look=std::clamp(speed*0.20f,55.0f,115.0f);
    glm::vec3 goal(0);
    auto findGoal=[&]{
        for(float distance:{look,look*0.5f,25.0f})
            if(SamplePath(c,l.s+l.dir*distance,goal)&&glm::length(glm::vec2(goal-l.pos))>12&&GroundConnection(l.pos,goal)&&(now>=l.blockedUntil||Distance(goal,l.blockedPos)>100))return true;
        return false;
    };
    bool ok=findGoal();
    if(!ok||stuck)
    {
        if(now>=l.nextSwitch || stuck)
        {
            if(SwitchPath(psw,l,l.fleeing?&pp:nullptr,true)){c=PcrvFromLo(l.path);ok=findGoal();}
            else {if(TrySlaiJump(psw,l,interested?&pp:nullptr,l.fleeing))return;l.dir=-l.dir;l.nextDirection=now+2.0f;ok=findGoal();}
            l.nextSwitch=now+0.5f;
        }
        else if(!ok){if(TrySlaiJump(psw,l,interested?&pp:nullptr,l.fleeing))return;l.dir=-l.dir;l.nextDirection=now+2.0f;ok=findGoal();}
    }
    if(!ok){hold();PlaySlai(l,false);return;}
    SlaiDrive(l,goal,speed,dt);
    const float actual=glm::length(glm::vec2(VelAloPos(a)));
    l.actualSpeed=glm::mix(l.actualSpeed,actual,std::min(dt*8,1.0f));
    if(l.actualSpeed<35 && now-l.lastProgress>1.0f){PlaySlai(l,false);return;}
    const bool running=l.fleeing&&l.run;
    PlayClip(l,running?l.run:l.walk,true,running?std::clamp(l.actualSpeed/c_sSlaiRun,0.25f,1.15f):std::clamp(l.actualSpeed/c_sSlaiWalk,0.25f,1.15f));
}

// after the world's load, before its start: each guest's copy made (started with the world's own objects) and placed
void OnLoaded(SW* psw)
{
    s_alive.clear();
    s_pendingSlai.clear();
    s_spawningSlai = nullptr;
    s_readyPlayer = nullptr;
    s_readyAt = 0;
    if (!psw) return;
    s_serial = psw->serial;
    const std::string world = WorldNameOf(g_transition.iwrecCur);
    const auto guests = GuestsOf(world);
    for (const cane::mods::Guest* pg : guests)
    {
        if (WantsSlai(*pg)) { s_pendingSlai.push_back(pg); continue; }
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

// Use the exact rest-pose skin matrices captured by the kept-world loader.
// Never leave a skin pointed at a kept-world bone or at the original player.
void RestoreSlaiSkin(SW* psw,LO* root)
{
    std::unordered_map<const XF*,const XF*> map;
    std::vector<ALO*> parts;
    for(const auto& up:psw->aploAll)
    {
        LO* o=up.get();if(!o||PloModSlaiNamespace(o)!=root||!o->ploCloneOf||!FCidDerivedFrom(o->Cid(),0x30))continue;
        auto* dst=static_cast<ALO*>(o);const auto* src=static_cast<const ALO*>(o->ploCloneOf);parts.push_back(dst);
        for(size_t i=0;i<src->axf.size()&&i<dst->axf.size();++i)map[src->axf[i].get()]=dst->axf[i].get();
    }
    size_t count=0,missing=0;
    for(ALO* dst:parts)
    {
        const auto* src=static_cast<const ALO*>(dst->ploCloneOf);
        const auto* original=PaabindKept(src);if(!original)continue;
        auto copy=*original;
        bool complete=true;
        for(auto& group:copy)for(auto& b:group)if(b.pxfBone)
        {
            auto it=map.find(b.pxfBone);
            if(it==map.end()){complete=false;++missing;}else {b.pxfBone=it->second;++count;}
        }
        if(complete)SetAloBoneBindings(dst,copy); // leave the namespace-checked default if incomplete
    }
    std::printf("Slai: original skin bindings mapped %zu, missing %zu\n",count,missing);
}

void StartSlaiGuest(SW* psw, LIVE& l)
{
    if (l.plo->fRemoved || !l.plo->fInWorld) AddLo(psw, l.plo);
    SetLoSt(psw, l.plo, 1);
    ALO* palo = static_cast<ALO*>(l.plo);
    RestoreSlaiSkin(psw,l.plo);
    const SOST* authored=l.plo->ploCloneOf ? Psost(static_cast<ALO*>(l.plo->ploCloneOf)) : nullptr;
    const glm::vec3 gravity=authored && authored->dvGravity.z < -1 ? authored->dvGravity : glm::vec3(0,0,-1750.0f);
    if (STEP* st=PstepFromLo(l.plo))
    {
        st->fNoContacts=0;st->nMotion=0;st->vecVel=glm::vec3(0);
        if(st->isoeCustom<0||size_t(st->isoeCustom)>=palo->asoe.size()) { LoadStepPhysBase(palo); PostLoadStepPhys(palo); }
        if(st->isoeCustom>=0&&size_t(st->isoeCustom)<palo->asoe.size())SetSoeOn(palo,&palo->asoe[st->isoeCustom]);
    }
    if(XF* x=PxfPos(palo))SetXfMode(palo,x,1);
    if(XF* x=PxfRot(palo))SetXfMode(palo,x,1);
    SetAloCnstrLin(palo,0); // free translation including gravity
    SetAloCnstrAng(palo,4); // yaw only: no uncontrolled tumbling
    SetAloGravity(palo,gravity);
    const int iwrec = IwrecFrom(*l.pg);
    l.walk = dynamic_cast<ASEG*>(PloKeptFind(iwrec, 277, 0x6D));
    l.idle = dynamic_cast<ASEG*>(PloKeptFind(iwrec, 4576, 0x6D));
    auto clip = [&](int oid) { return dynamic_cast<ASEG*>(PloKeptFind(iwrec, oid, 0x6D)); };
    l.run=clip(126); l.jumpClip=clip(152); l.doubleJumpClip=clip(160); l.hurt=clip(222); l.shock=clip(223); l.defeat=clip(231);
    l.combatIdle=clip(5568); l.combatWalk=clip(129);
    for(int oid : {4576,4560,5543,5546,5552,5539}) if(auto* a=clip(oid)) l.idles.push_back(a);
    for(int oid : {291,290,288,287}) if(auto* a=clip(oid)) l.attacks.push_back(a);
    if (ACTOR* pa=PactorFromLo(l.plo)) { SetDmgCapacity(pa,c_nSlaiMaxHP); SetDmgCounter(pa,&l.hp); }
    if (l.pg->patrol >= 0) Place(l.plo, *l.pg);
    // The automatic guest uses a nearby patrol, never the manifest's dummy origin.
    STEP* step = PstepFromLo(l.plo);
    l.pos = SlaiFeet(l.plo);
    l.heading = step ? step->radHeading : glm::radians(l.pg->heading);
    l.fPosition = true;
    l.nextSearch = g_clock.t;
    std::printf("Slai: independent guest; walk %d idle %d\n", l.walk ? l.walk->oid : -1, l.idle ? l.idle->oid : -1);
    if (!l.walk) std::fprintf(stderr, "Slai: missing ASEG 277 in source world; stays idle\n");
    std::printf("Slai: root collision shapes %zu, IK pairs %zu, floor mesh %d, gravity %.1f\n",palo->asoe.size(),palo->apair.size(),step?step->isoeCustom:-1,Psost(palo)?Psost(palo)->dvGravity.z:0.0f);
    PlaySlai(l, false);
}

// A player template carries mission script frames and object aliases. Keep only the isolated copy
// in a private namespace and register it BEFORE its native startup.
void PrepareSlaiCopy(SW* psw, LO* root, const std::vector<LO*>& objects)
{
    s_alive.push_back({s_spawningSlai, root});
    s_alive.back().fSlai = true;
    std::unordered_map<const XF*,XF*> xfmap;
    for (LO* obj : objects)
    {
        obj->ploNsRow = root;
        obj->pframe.reset();
        obj->psidebag.reset();
        if (!obj->ploCloneOf || !FCidDerivedFrom(obj->Cid(),0x30)) continue;
        const auto* src=static_cast<const ALO*>(obj->ploCloneOf);
        auto* dst=static_cast<ALO*>(obj);
        if(obj==root) FCopyKeptStep(dst,src);
        for (size_t i=0;i<src->axf.size() && i<dst->axf.size();++i)
            xfmap[src->axf[i].get()]=dst->axf[i].get();
    }
    auto remap=[&](XF* p)->XF* {
        auto it=xfmap.find(p); return it==xfmap.end()?p:it->second;
    };
    for (LO* obj : objects)
    {
        if (!obj->ploCloneOf || !FCidDerivedFrom(obj->Cid(),0x30)) continue;
        const auto* src=static_cast<const ALO*>(obj->ploCloneOf);
        auto* dst=static_cast<ALO*>(obj);
        for (auto& xf:dst->axf) { xf->pxfRef=remap(xf->pxfRef); xf->pxfOrigin=remap(xf->pxfOrigin); }
        for (auto& xf:dst->apxfRef) xf=remap(xf);
        for (auto& part:dst->apartxf) part.second=remap(part.second);
        for (auto& soe:dst->asoe) soe.pxf=remap(soe.pxf);
        for (auto& link:dst->alink) { link.pxf=remap(link.pxf); link.pxfTarget=remap(link.pxfTarget); }
        for (auto& chain:dst->achain) for (auto& e:chain.ae) e.pxf=remap(e.pxf);
        // CloneAloData omits the authored two-bone IK pairs. Restore only fully local pairs.
        for (const auto& pair:src->apair)
        {
            bool local=true;
            for (auto* p:pair.apxf) if (p && !xfmap.count(p)) local=false;
            if (!local) continue;
            auto q=pair;
            for (auto& p:q.apxf) p=remap(p);
            q.ptrkgWeight=nullptr; q.ptWeight=nullptr; // preserve authored rest twist
            dst->apair.push_back(q);
        }
    }
}

void TrySpawnSlai(SW* psw)
{
    if (s_pendingSlai.empty()) return;
    LO* player = PloPoCurrent(psw);
    if (!player || !player->fInWorld || player->fRemoved || !FCidDerivedFrom(player->Cid(), 0x34) || NPoEnabled(player) != 1 || FGamePausedForAi() || FSlaiSafeHouseOpen() || g_gamepause.c78 > 0)
    { s_readyPlayer = nullptr; return; }
    if (player != s_readyPlayer) { s_readyPlayer = player; s_readyAt = g_clock.t + 3.0f; return; }
    if (g_clock.t < s_readyAt) return;
    const auto* pg = s_pendingSlai.front();
    if (pg->patrol < 0) { float s=0; if (!FindPatrol(psw,PosAloCenter(static_cast<ALO*>(player)),s)) { s_readyAt=g_clock.t+1; return; } }
    const int iwrec = IwrecFrom(*pg);
    if (iwrec >= 0 && PckwsKeptWorld(iwrec) == PCKWS_Loading) return;
    s_pendingSlai.erase(s_pendingSlai.begin());
    LO* tmpl = iwrec >= 0 ? PloKeptFind(iwrec, pg->object, 0x35) : nullptr;
    if (!tmpl) { std::fprintf(stderr, "Slai: deferred spawn failed; template unavailable\n"); return; }
    std::printf("Slai: deferred spawn after player selection (player oid %d)\n", player->oid);
    s_spawningSlai = pg;
    LO* clone = PloPcCloneIntoWorld(psw, tmpl, true, PrepareSlaiCopy);
    s_spawningSlai = nullptr;
    if (!clone) return;
    InitSlaiClone(psw, clone);
    StartSlaiGuest(psw, s_alive.back());
    StepSlai(psw, s_alive.back());
    std::printf("Slai: deferred spawn complete; current player preserved: %d\n", PloPoCurrent(psw) == player);
}

// after the start: a guard's patrol (a path of this world, walked along and back, again and again)
void OnStarted(SW* psw)
{
    if (!psw || psw->serial != s_serial) return;
    for (LIVE& l : s_alive)
    {
        if (l.fSlai)
        {
            StartSlaiGuest(psw, l);
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
    TrySpawnSlai(psw);
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
            if(l.fSlai) if(STEP* st=PstepFromLo(l.plo))
                std::printf("Slai: floor %d, path %d, velocity %.1f, hp %d, fleeing %d\n",st->pxpFloor!=nullptr,l.path?l.path->oid:-1,glm::length(glm::vec2(VelAloPos(st->palo))),l.hp,l.fleeing);
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
const LO* PloModSlaiNamespace(const LO* plo)
{
    if (!plo) return nullptr;
    if (FModSlaiGuest(plo)) return plo;
    if (plo->ploNsRow && FModSlaiGuest(plo->ploNsRow)) return plo->ploNsRow;
    return nullptr;
}
bool FModSlaiOrdinaryGuard(const LO* plo)
{
    if (!plo || !FCidDerivedFrom(plo->Cid(),0x38)) return false;
    // pcguard.cpp lists root 11539 as the ordinary flashlight guard across episodes.
    const CID cid=plo->Cid();
    if(cid==0x3B||cid==0x3C||cid==0x3F||cid==0x40||FCidDerivedFrom(cid,0x3B))return false;
    if(plo->oid==11539 || (plo->ploCloneOf && plo->ploCloneOf->oid==11539))return true;
    // Exclude named boss/special/script actors. The editor identifies ordinary guards
    // and swarmers by the authored shared-group name, not only by the base CID.
    if (g_psw && plo->igrp>=0 && size_t(plo->igrp)<g_psw->awe.size())
    {
        const WE& group=g_psw->awe[size_t(plo->igrp)];
        // Proxies can have their own instance OID; use the shared group's loaded root too.
        for(int i=0;i<group.cloLoaded && size_t(i)<group.aplo.size();++i){const LO* root=group.aplo[size_t(i)];if(root&&root->oid==11539&&root->Cid()==cid)return true;}
        std::string name=group.StrName();
        for(char& ch:name) ch=char(std::tolower(static_cast<unsigned char>(ch)));
        if(name.find("flashlight")!=std::string::npos || name.find("searchlight")!=std::string::npos)return true;
        if(name.find("boss")!=std::string::npos || name.find("carmelita")!=std::string::npos ||
           name.find("neyla")!=std::string::npos || name.find("dimitri")!=std::string::npos ||
           name.find("rajan")!=std::string::npos || name.find("contessa")!=std::string::npos) return false;
        // Known special scripted subclasses are intentionally excluded even with a matching name.
        if(name.find("swarm")!=std::string::npos) return true;
        if(plo->Cid()==0x3A || plo->Cid()==0x3B || plo->Cid()==0x3C || plo->Cid()==0x3F || plo->Cid()==0x40) return false;
        if(name.find("guard")!=std::string::npos) return true;
    }
    return plo->Cid()==0x38 || plo->Cid()==0x39;
}

bool FModSlaiTargetable(const LO* plo)
{
    for (const LIVE& l : s_alive) if (l.fSlai && l.plo == plo)
        return plo && plo->fInWorld && !plo->fRemoved && l.hasSpawn && l.hp > 0;
    return false;
}
const char* SzModSlaiState(const LO* plo)
{
    for(const LIVE& l:s_alive)if(l.fSlai&&l.plo==plo)return l.state;
    return "?";
}
int* PnModSlaiHealth(const LO* plo)
{
    for (LIVE& l : s_alive) if (l.fSlai && l.plo == plo) return &l.hp;
    return nullptr;
}

int NModSlaiCoins(const LO* plo)
{
    for(const LIVE& l:s_alive)if(l.fSlai&&l.plo==plo)return l.coins;
    return 0;
}
bool FModSlaiPickupsActive()
{
    if(!s_pendingSlai.empty())return true;
    for(const LIVE& l:s_alive)if(l.fSlai)return true;
    return false;
}
bool FModSlaiWantsPickup(const LO* plo,bool heal)
{
    if(!plo||!FModSlaiTargetable(plo))return false;
    for(const LIVE& l:s_alive)if(l.fSlai&&l.plo==plo)
        return g_clock.t>=l.hurtUntil && (heal?l.hp<c_nSlaiMaxHP:l.coins<c_nSlaiMaxCoins);
    return false;
}
bool FModSlaiTakePickup(LO* plo,bool heal,const glm::vec3& pos)
{
    if(!FModSlaiWantsPickup(plo,heal)||!SlaiClearLine(SlaiFeet(plo)+glm::vec3(0,0,65),pos))return false;
    for(LIVE& l:s_alive)if(l.fSlai&&l.plo==plo){
        if(heal){const int before=l.hp;l.hp=std::min(c_nSlaiMaxHP,l.hp+25);
            std::printf("Slai: charm HP %d -> %d / %d\n",before,l.hp,c_nSlaiMaxHP);}
        else{++l.coins;std::printf("Slai: coin wallet %d / %d\n",l.coins,c_nSlaiMaxCoins);}
        l.pickupUntil=0;l.pickupStarted=0;l.nextPickupSearch=g_clock.t;return true;
    }
    return false;
}

// Called before native player damage/mission transitions. Slai owns his reaction and respawn.
bool FModSlaiAcceptDamage(LO* plo, const CTMSG* hit)
{
    if (!hit || !hit->ploSender || hit->ploSender == plo) return false;
    const bool electric=FPcCarmProjectile(hit->ploSender);
    for (LIVE& l : s_alive) if (l.fSlai && l.plo == plo)
    {
        const char* rejected=nullptr;
        if (!l.hasSpawn || l.hp<=0) rejected="not active";
        else if (FGamePausedForAi()) rejected="paused";
        else if (g_clock.t<l.nextHit) rejected="immunity";
        else if(hit->nState>=0 || hit->grfFilter<=1 || hit->grfFilter==0x100000 || hit->grfFilter==0x80000) rejected="not damage";
        if(rejected)
        {
            if(electric || std::getenv("SLY2N_SLAIGUARD"))
                std::printf("Slai: ignored hit from %d: %s (state %d filter 0x%x HP %d)\n",hit->ploSender->oid,rejected,hit->nState,hit->grfFilter,l.hp);
            return false;
        }
        // Scale Carmelita contacts to Slai's own HP; preserve other authored hits.
        const int before=l.hp;
        const int64_t damage=electric?20:-int64_t(hit->nState);
        l.hp -= int(std::min<int64_t>(l.hp,damage));
        std::printf("Slai: hit from %d, %s, damage %d, HP %d -> %d / %d\n",hit->ploSender->oid,electric?"electric":"normal",before-l.hp,before,l.hp,c_nSlaiMaxHP);
        l.nextHit=g_clock.t+1.0f; l.attackUntil=0; l.restUntil=0;
        if(l.visiting||l.following||l.followHolding)ResetSocial(l,g_clock.t,true);
        if(STEP* step=PstepFromLo(l.plo)) step->vecVel=glm::vec3(0);
        LO* attacker=hit->ploSender;
        if(!LiveGuard(attacker)&&FCidDerivedFrom(attacker->Cid(),0x4F))attacker=PloProjectileLauncherActor(attacker);
        if(LiveGuard(attacker)){l.attackTarget=attacker;l.lastGuardSeen=g_clock.t;l.nextCombat=0;}
        if (l.hp <= 0) { l.respawnAt=g_clock.t+6.0f; RestartClip(l,l.defeat); }
        else { ASEG* reaction=electric && l.shock?l.shock:l.hurt; l.hurtUntil=g_clock.t+ClipTime(reaction,0.6f);
            if(!electric)l.nextHit=std::max(l.nextHit,l.hurtUntil+0.75f); // give him a window to act after enemy hits
            l.nextRest=g_clock.t+SlaiExploreTime(); RestartClip(l,reaction); }
        return true;
    }
    return false;
}

// Experimental fallback for Carmelita's in-game projectile against a Slai guest only.
LO* PloModSlaiShotContact(const glm::vec3& a,const glm::vec3& b,glm::vec3* hit)
{
    const glm::vec3 v=b-a; const float len2=glm::dot(v,v);
    if (len2<0.0001f) return nullptr;
    LO* best=nullptr; float tbest=2.0f;
    for (const LIVE& l:s_alive) if (FModSlaiTargetable(l.plo))
    {
        // Approximate upright torso using a small set of overlapping spheres at the feet position.
        for(float z:{30.0f,70.0f,110.0f})
        {
            const glm::vec3 center=SlaiFeet(l.plo)+glm::vec3(0,0,z), w=a-center;
            const float r=35.0f, B=glm::dot(w,v), C=glm::dot(w,w)-r*r;
            const float disc=B*B-len2*C;
            if(disc<0) continue;
            const float t=C<=0?0.0f:(-B-std::sqrt(disc))/len2;
            if(t<0||t>1||t>=tbest) continue;
            const glm::vec3 q=a+t*v;
            if(!SlaiClearLine(a,q)) continue;
            best=l.plo; tbest=t; if(hit)*hit=q;
        }
    }
    return best;
}
