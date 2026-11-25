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
    ASEG* jumpClip = nullptr;
    ASEG* hurt = nullptr;
    ASEG* shock = nullptr;
    ASEG* combatIdle = nullptr;
    ASEG* combatWalk = nullptr;
    ASEG* defeat = nullptr;
    ASEG* playing = nullptr;
    std::vector<ASEG*> idles, attacks;
    ASEG* restClip = nullptr;
    LO* attackTarget = nullptr;
    float lastGuardSeen = 0;
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
    float routeSince = 0, lastProgress = 0, nextProgressCheck = 0, nextDirection = 0;
    glm::vec3 progressPos{0.0f}, safePos{0.0f};
    bool hasSafePos = false;
    std::deque<LO*> recentPaths;
    float actualSpeed = 0.0f;
    // A bounded jump owns locomotion until actual touchdown, never the curve height.
    bool jumping = false;
    float jumpStarted = 0, jumpDeadline = 0, nextJump = 0;
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
constexpr float c_gSlaiVisual = 0.70710678f; // 90 degree horizontal FOV, character heading, not camera
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
bool GroundConnection(const glm::vec3& from,const glm::vec3& to)
{
    glm::vec3 last=from;
    const float dist=glm::length(glm::vec2(to-from));
    const int count=std::clamp(int(std::ceil(dist/30.0f)),1,12);
    for(int i=1;i<=count;++i)
    {
        glm::vec3 q=glm::mix(from,to,float(i)/count);
        q.z=last.z;
        if(!FDropOnGround(&q,50,-70,false)||std::abs(q.z-last.z)>35) return false;
        if(!SlaiClearLine(last+glm::vec3(0,0,65),q+glm::vec3(0,0,65))) return false;
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
    else v=glm::vec3(0);
    // STEP's floor solver uses negative desired velocity in the heading frame (as GUARD does).
    // Keep vertical integration, floor contacts, collision response and angular braking native.
    step->nMotion=0; step->fNoContacts=0;
    SetStepPhysHeading(a,l.heading);
    step->vecVel=-glm::transpose(step->matHeading)*v;
    WakeAlo(a);
}
bool SamplePath(const CRV* c,float s,glm::vec3& q)
{
    if(!c || c->as.size()<2)return false;
    glm::vec3 tangent(0);
    s=c->fClosed?SWrapCrv(c,s):std::clamp(s,c->as.front(),c->as.back());
    EvalCrvS(c,s,&q,&tangent);
    return std::isfinite(q.x)&&std::isfinite(q.y)&&std::isfinite(q.z)&&FDropOnGround(&q,100,-250,false);
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
    RememberPath(l,l.path);l.path=best;l.s=bestS;l.dir=bestDir;l.routeSince=g_clock.t;l.lastProgress=g_clock.t;l.nextDirection=g_clock.t+1.5f;
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
bool PlanSlaiJump(LIVE& l, LO* path, float s, int dir, SlaijumpPlan& out)
{
    const CRV* c=PcrvFromLo(path);
    glm::vec3 q(0);
    if(!SamplePath(c,s,q))return false;
    const glm::vec3 delta=q-l.pos;
    const float span=glm::length(glm::vec2(delta));
    if(span<65 || span>350 || delta.z>110 || delta.z<-550)return false;
    // A normal walking connection does not need a jump.
    if(GroundConnection(l.pos,q))return false;
    for(glm::vec2 offset:{glm::vec2(22,0),glm::vec2(-22,0),glm::vec2(0,22),glm::vec2(0,-22)})
    {
        glm::vec3 test=q+glm::vec3(offset,0);
        if(!FDropOnGround(&test,35,-35,false)||std::abs(test.z-q.z)>25)return false;
    }
    glm::vec3 ahead(0);
    if(!SamplePath(c,s+dir*65,ahead)||glm::length(glm::vec2(ahead-q))<15||!GroundConnection(q,ahead))return false;
    const SOST* st=Psost(static_cast<ALO*>(l.plo));
    if(!st||!std::isfinite(st->dvGravity.z)||st->dvGravity.z>=-1)return false;
    const float g=-st->dvGravity.z;
    // Choose a small arc for a step up or down. Apply only the initial velocity;
    // native STEP physics, gravity and collision response integrate the flight.
    const float rise=std::max(0.0f,delta.z)+65.0f;
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
    out={path,q,vel,BoundS(c,s),t,dir};return true;
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
    if(!best.path)return false;
    l.jumping=true;l.jumpStarted=now;l.jumpDeadline=now+best.flight+0.8f;
    l.jumpPath=best.path;l.jumpGoal=best.target;l.jumpS=best.s;l.jumpDir=best.dir;
    l.heading=std::atan2(best.velocity.y,best.velocity.x);
    SetStepPhysHeading(st->palo,l.heading);
    st->nMotion=-1;st->vecVel=glm::vec3(0);st->pxpFloor=nullptr;
    SetAloVelocityTo(st->palo,best.velocity);WakeAlo(st->palo);
    RestartClip(l,l.jumpClip);
    std::printf("Slai: jump to path %d, height %.0f, flight %.2f\n",best.path->oid,best.target.z-l.pos.z,best.flight);
    return true;
}
bool StepSlaiJump(SW* psw,LIVE& l,STEP* st,float now)
{
    if(!l.jumping)return false;
    const float vz=VelAloPos(st->palo).z;
    glm::vec3 q=l.pos;
    const bool landed=now-l.jumpStarted>0.18f&&vz<100&&st->pxpFloor&&FDropOnGround(&q,30,-35,false)&&std::abs(q.z-l.pos.z)<30;
    if(!landed&&now<l.jumpDeadline)
    {
        st->nMotion=-1;st->vecVel=glm::vec3(0);
        return true; // do not reapply launch velocity, hold, or snap to path
    }
    l.jumping=false;st->nMotion=0;st->vecVel=glm::vec3(0);
    l.lastProgress=now;l.nextProgressCheck=now+1;l.progressPos=l.pos;l.nextJump=now+3;
    if(landed)
    {
        const bool arrived=Distance(l.pos,l.jumpGoal)<110;
        if(arrived){RememberPath(l,l.path);l.path=l.jumpPath;l.s=l.jumpS;l.dir=l.jumpDir;}
        else {float s=0;LO* path=FindPatrol(psw,l.pos,s,nullptr,150);if(path){RememberPath(l,l.path);l.path=path;l.s=s;}else l.path=nullptr;}
        l.safePos=l.pos;l.hasSafePos=true;l.routeSince=now;
        std::printf("Slai: jump landed (%s)\n",arrived?"target":"other floor");
    }
    else {l.path=nullptr;l.nextSearch=now;std::printf("Slai: jump timeout\n");}
    l.jumpPath=nullptr;
    return false;
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
    LO* best = nullptr; float dist = 650.0f;
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
        if (FDropOnGround(&q,100,-250,false)) l.s=trial;
    }
    std::printf("Slai: path %d, start %.1f, length %.1f, walk %d\n", path->oid, l.s, c->as.back() - c->as.front(), l.walk->oid);
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
        if(l.respawnAt==0){l.respawnAt=now+6;RestartClip(l,l.defeat);}
        if(now<l.respawnAt){hold();return;}
        std::printf("Slai: respawn, HP 0 -> 400\n");
        l.hp=400;l.respawnAt=0;l.hurtUntil=0;l.nextHit=now+2;
        l.attackUntil=0;l.attackTarget=nullptr;l.fleeing=false;l.restUntil=0;
        l.jumping=false;l.jumpPath=nullptr;l.nextJump=now+3;
        l.path=l.spawnPath;l.s=l.spawnS;l.pos=l.spawn;l.heading=l.spawnHeading;
        l.recentPaths.clear();l.routeSince=now;l.lastProgress=now;l.nextProgressCheck=now+1;l.progressPos=l.spawn;
        l.nextRest=now+RandomRange(5,10);l.nextSearch=now;l.nextSwitch=now+3;
        PlaceAt(l.plo,l.spawn,l.heading);PlaySlai(l,false);hold();return;
    }
    if(!l.hasSpawn)
    {
        if(now<l.nextSearch){hold();return;}l.nextSearch=now+1;
        if(!StartSlaiPath(psw,l)){hold();PlaySlai(l,false);return;}
        glm::vec3 q(0);if(!SamplePath(PcrvFromLo(l.path),l.s,q)){l.path=nullptr;hold();return;}
        l.hasSpawn=true;l.spawn=q;l.spawnS=l.s;l.spawnPath=l.path;l.spawnHeading=l.heading;
        l.pos=q;l.safePos=q;l.hasSafePos=true;l.progressPos=q;l.lastProgress=now;l.nextProgressCheck=now+1;
        l.routeSince=now;l.nextRest=now+RandomRange(5,10);l.nextSwitch=now+3;
        PlaceAt(l.plo,q,l.heading);hold();return;
    }
    if(StepSlaiJump(psw,l,step,now))return;
    // Retain a last grounded position only for genuine falls, not per-frame placement.
    if(step->pxpFloor){l.safePos=l.pos;l.hasSafePos=true;}
    if(l.hasSafePos && !step->pxpFloor && l.pos.z<l.safePos.z-350)
    {
        l.pos=l.safePos;PlaceAt(l.plo,l.pos,l.heading);l.path=nullptr;l.nextSearch=now;l.lastProgress=now;hold();return;
    }
    if(now<l.hurtUntil){hold();return;}
    ScanSlaiPads(psw,l);
    if(!l.path && now>=l.nextSearch){l.nextSearch=now+0.5f;if(StartSlaiPath(psw,l)){l.routeSince=now;l.lastProgress=now;}}
    const CRV* c=PcrvFromLo(l.path);
    if(!c||c->as.size()<2){l.path=nullptr;hold();PlaySlai(l,false);return;}
    // Resynchronize route progress to actual physical position, not an assumed kinematic distance.
    float nearS=l.s;glm::vec3 raw(0);NearestCrvAll(c,l.pos,&raw,nullptr,nullptr,&nearS);
    if(std::isfinite(nearS))l.s=BoundS(c,nearS);
    LO* player=PloPoCurrent(psw);
    const bool carm=player&&player!=l.plo&&FPcCarm(player)&&FCidDerivedFrom(player->Cid(),0x30);
    glm::vec3 pp=carm?PosAloCenter(static_cast<ALO*>(player)):glm::vec3(0);
    const glm::vec3 eye=l.pos+glm::vec3(0,0,65);
    const float d=carm?Distance(pp,eye):1e10f;
    const bool seen=carm&&d<=c_sSlaiDetect&&SlaiClearLine(eye,pp);
    bool discovered=false;
    if(seen&&d<=c_sSlaiObserve)if(STEP* st=PstepFromLo(player))
    {
        const float rad=RadStepHeading(st);const glm::vec2 f(std::cos(rad),std::sin(rad)),to(eye.x-pp.x,eye.y-pp.y);
        const float t2=glm::dot(to,to);discovered=t2<1||glm::dot(f,to)>=c_gSlaiVisual*std::sqrt(t2);
    }
    if(!carm||d>=c_sSlaiSafe)l.fleeing=false;
    else if(discovered&&!l.fleeing){l.fleeing=true;l.nextDirection=now;}
    if(l.fleeing){l.attackUntil=0;l.attackTarget=nullptr;l.restUntil=0;}
    // Combat persists through attack cooldowns; only an actual approach can translate the actor.
    if(!l.fleeing)
    {
        if(!LiveGuard(l.attackTarget)){l.attackTarget=nullptr;l.attackUntil=0;}
        if(!l.attackTarget&&!l.attacks.empty()){l.attackTarget=NearbyGuard(l.pos);if(l.attackTarget)l.lastGuardSeen=now;}
        if(l.attackTarget)
        {
            const glm::vec3 target=PosAloCenter(static_cast<ALO*>(l.attackTarget));const float dist=Distance(eye,target);
            const bool clear=SlaiClearLine(eye,target);if(clear)l.lastGuardSeen=now;
            if(dist>950||now-l.lastGuardSeen>2){l.attackTarget=nullptr;l.attackUntil=0;}
            else
            {
                l.restUntil=0;l.lastProgress=now;FaceSlai(l,target,dt);
                if(now<l.attackUntil){if(!l.struck&&now>=l.strikeAt){l.struck=true;StrikeSlai(l);}hold();return;}
                const glm::vec3 v=target-l.pos;const float angle=std::abs(std::remainder(std::atan2(v.y,v.x)-RadStepHeading(step),6.2831853f));
                if(clear&&dist<=190&&angle<0.30f&&now>=l.nextAttack&&!l.attacks.empty())
                {
                    ASEG* clip=l.attacks[std::uniform_int_distribution<size_t>(0,l.attacks.size()-1)(s_rng)];
                    const float duration=ClipTime(clip,0.8f);l.struck=false;l.attackUntil=now+duration;l.strikeAt=now+duration*0.45f;l.nextAttack=l.attackUntil+1.5f;
                    RestartClip(l,clip);hold();return;
                }
                bool approach=false;glm::vec3 q=l.pos;
                if(clear&&dist>165)
                {
                    glm::vec2 dir=glm::vec2(target-l.pos);float len=glm::length(dir);
                    if(len>1){dir/=len;q=l.pos+glm::vec3(dir*std::min(100.0f,len-std::min(150.0f,len)),0);approach=GroundConnection(l.pos,q);}
                }
                if(approach)SlaiDrive(l,q,c_sSlaiWalk,dt);else hold();
                PlayClip(l,approach?(l.combatWalk?l.combatWalk:l.walk):(l.combatIdle?l.combatIdle:l.idle));return;
            }
        }
    }
    float speed=c_sSlaiWalk;bool move=true;
    if(l.fleeing||seen)
    {
        l.restUntil=0;l.nextRest=now+RandomRange(5,10);
        if(l.fleeing)speed=l.run?c_sSlaiRun:c_sSlaiWalk;
        else if(d<=c_sSlaiObserve){move=false;FaceSlai(l,pp,dt);}
        else if(d<c_sSlaiObserve+150)speed=c_sSlaiWalk;
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
        if(now>=l.nextRest&&now>=l.restUntil){l.restUntil=now+RandomRange(5,12);l.nextRest=l.restUntil+RandomRange(5,10);l.restClip=l.idles.empty()?l.idle:l.idles[std::uniform_int_distribution<size_t>(0,l.idles.size()-1)(s_rng)];}
        if(now<l.restUntil){hold();PlayClip(l,l.restClip);l.lastProgress=now;return;}
    }
    if(!move){hold();PlaySlai(l,false);l.lastProgress=now;return;}
    if(now>=l.nextProgressCheck)
    {
        if(glm::length(glm::vec2(l.pos-l.progressPos))>45)l.lastProgress=now;
        l.progressPos=l.pos;l.nextProgressCheck=now+1;
    }
    const bool stuck=now-l.lastProgress>3.0f, old=now-l.routeSince>30.0f;
    if(now>=l.nextSwitch)
    {
        l.nextSwitch=now+0.5f;
        if((now-l.routeSince>3||stuck||l.fleeing)&&SwitchPath(psw,l,l.fleeing?&pp:nullptr,old||stuck))
        {c=PcrvFromLo(l.path);l.nextSwitch=now+3;}
    }
    if(!c){hold();return;}
    const float look=std::clamp(speed*0.20f,55.0f,115.0f);
    glm::vec3 goal(0);
    auto findGoal=[&]{
        for(float distance:{look,look*0.5f,25.0f})
            if(SamplePath(c,l.s+l.dir*distance,goal)&&glm::length(glm::vec2(goal-l.pos))>12&&GroundConnection(l.pos,goal))return true;
        return false;
    };
    bool ok=findGoal();
    if(!ok||stuck)
    {
        if(now>=l.nextSwitch || stuck)
        {
            if(SwitchPath(psw,l,l.fleeing?&pp:nullptr,true)){c=PcrvFromLo(l.path);ok=findGoal();}
            else {if(TrySlaiJump(psw,l,seen||l.fleeing?&pp:nullptr,l.fleeing))return;l.dir=-l.dir;l.nextDirection=now+2.0f;ok=findGoal();}
            l.lastProgress=now;l.nextSwitch=now+0.5f;
        }
        else if(!ok){if(TrySlaiJump(psw,l,seen||l.fleeing?&pp:nullptr,l.fleeing))return;l.dir=-l.dir;l.nextDirection=now+2.0f;ok=findGoal();}
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
    l.run=clip(126); l.jumpClip=clip(152); l.hurt=clip(222); l.shock=clip(223); l.defeat=clip(231);
    l.combatIdle=clip(5568); l.combatWalk=clip(129);
    for(int oid : {4576,4560,5543,5546,5552,5539}) if(auto* a=clip(oid)) l.idles.push_back(a);
    for(int oid : {291,290,288,287}) if(auto* a=clip(oid)) l.attacks.push_back(a);
    if (ACTOR* pa=PactorFromLo(l.plo)) { SetDmgCapacity(pa,400); SetDmgCounter(pa,&l.hp); }
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
    // Exclude named boss/special/script actors. The editor identifies ordinary guards
    // and swarmers by the authored shared-group name, not only by the base CID.
    if (g_psw && plo->igrp>=0 && size_t(plo->igrp)<g_psw->awe.size())
    {
        std::string name=g_psw->awe[size_t(plo->igrp)].StrName();
        for(char& ch:name) ch=char(std::tolower(static_cast<unsigned char>(ch)));
        if(name.find("boss")!=std::string::npos || name.find("carmelita")!=std::string::npos ||
           name.find("neyla")!=std::string::npos || name.find("dimitri")!=std::string::npos ||
           name.find("rajan")!=std::string::npos || name.find("contessa")!=std::string::npos) return false;
        // Known special scripted subclasses are intentionally excluded even with a matching name.
        if(plo->Cid()==0x3A || plo->Cid()==0x3B || plo->Cid()==0x3C || plo->Cid()==0x3F || plo->Cid()==0x40) return false;
        if(name.find("guard")!=std::string::npos || name.find("swarm")!=std::string::npos) return true;
    }
    return plo->Cid()==0x38 || plo->Cid()==0x39;
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
        // Preserve the authored projectile/contact damage, not a new fixed value.
        const int before=l.hp;
        const int64_t damage=-int64_t(hit->nState);
        l.hp -= int(std::min<int64_t>(l.hp,damage));
        std::printf("Slai: hit from %d, %s, damage %d, HP %d -> %d / 400\n",hit->ploSender->oid,electric?"electric":"normal",before-l.hp,before,l.hp);
        l.nextHit=g_clock.t+1.0f; l.attackUntil=0; l.restUntil=0;
        if(STEP* step=PstepFromLo(l.plo)) step->vecVel=glm::vec3(0);
        if (LiveGuard(hit->ploSender)) { l.attackTarget=hit->ploSender; l.lastGuardSeen=g_clock.t; }
        if (l.hp <= 0) { l.respawnAt=g_clock.t+6.0f; RestartClip(l,l.defeat); }
        else { ASEG* reaction=electric && l.shock?l.shock:l.hurt; l.hurtUntil=g_clock.t+ClipTime(reaction,0.6f); RestartClip(l,reaction); }
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
