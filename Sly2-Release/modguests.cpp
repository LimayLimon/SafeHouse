// See modguests.h.
#include "modguests.h"

#include "activate.h"
#include "ai.h"
#include "aijobs.h"
#include "nav.h"
#include "navlink.h"
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
#include <utility>

// Existing public puppet animation accessor (implemented in pccharacter.cpp).
int OidPcPuppetAnim();
bool FPcGuardMoves();

namespace
{
struct SlaiNavNode { glm::vec3 pos; int parent; };
struct SlaiPatrolEntry { LO* path; float s; int dir; glm::vec3 pos; float score; };
// Short-term spatial memory. Jump failures are distinct from walking failures.
struct SlaiFailedGoal { glm::vec3 pos; float until; bool jump; };
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
    std::deque<SlaiFailedGoal> failedGoals;
    // Native walkable-area route, cached as positions rather than engine pointers.
    std::deque<glm::vec3> meshRoute;
    glm::vec3 meshThreat{0};
    float meshRetry=0, meshUntil=0, meshCheck=0;
    int meshSerial=-1, meshMode=-1;
    bool meshJump=false;
    int meshGeneration=-1;
    glm::mat3 floorFrame{1.f};
    float nextFloorFit=0.f, floorFitHeading=0.f;
    bool floorFitValid=false;
    const KIND2* nativeJumpTiming=nullptr;
    glm::vec3 meshLanding{0};
    glm::vec3 commandedGoal{0};
    bool hasCommandedGoal=false;
    float nextReverse=0;
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
    ASEG* waterClip = nullptr;
    ASEG* binocularIn = nullptr;
    ASEG* binocularOut = nullptr;
    int binocularPhase = 0; // 0 inactive, 1 observe, 2 put away
    float binocularUntil = 0;
    LO* attackTarget = nullptr;
    float lastGuardSeen = 0, nextCombat = 0;
    float restUntil = 0, nextRest = 0, nextSwitch = 0;
    float hurtUntil = 0, respawnAt = 0, nextHit = 0;
    float nextGuardProjectileHit = 0; // shared by all ordinary guards, Slai only
    float attackUntil = 0, strikeAt = 0, nextAttack = 0;
    bool struck = false, fleeing = false;
    // Social activity is separate from mood and never overrides danger.
    LO* socialPlayer = nullptr;
    bool following = false, followHolding = false, visiting = false;
    float lastPlayerSeen = 0, holdUntil = 0, nextVisit = 0;
    float visitDeadline = 0, visitEnd = 0, socialCooldown = 0;
    float nextCuriosity = 0, followDeadline = 0;
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
    bool waterJump=false, waterCrossFlight=false;
    float nextWaterCrossProbe=0;
    // Independent bounded local navigation graph, built only when escape steering fails.
    std::vector<SlaiNavNode> navNodes;
    std::deque<glm::vec3> navRoute;
    size_t navCursor=0;
    float navNext=0, navCooldown=0, navStarted=0;
    std::deque<LO*> recentPaths;
    // Separate exploration planner: do not share the escape graph or its cooldown.
    std::vector<SlaiNavNode> patrolNodes;
    std::vector<SlaiPatrolEntry> patrolEntries;
    std::deque<glm::vec3> patrolRoute;
    size_t patrolCursor=0;
    LO* patrolTarget=nullptr;
    float patrolTargetS=0, patrolNext=0, patrolStarted=0, patrolRetry=0;
    int patrolTargetDir=1;
    float actualSpeed = 0.0f;
    // A bounded jump owns locomotion until actual touchdown, never the curve height.
    bool jumping = false, jumpRecoveryAirborne = false;
    float jumpStarted = 0, jumpDeadline = 0, nextJump = 0;
    // A high jump has one physical launch and one bounded mid-air impulse.
    bool roofSecondPending = false, roofFlight = false;
    float roofSecondAt = 0;
    glm::vec3 roofApex{0};
    LO* jumpPath = nullptr;
    glm::vec3 jumpGoal{0.0f};
    float jumpS = 0;
    int jumpDir = 1;
    // Local vertical link: walk to a checked launch point, then revalidate the arc.
    bool verticalApproach=false, verticalFlight=false, verticalAway=false;
    float verticalUntil=0, nextVertical=0;
    glm::vec3 verticalFrom{0}, verticalTo{0}, verticalIntent{0};
    const char* verticalState="jump";
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
constexpr float c_sSlaiObserve = 1400.0f; // original Carmelita detection range
constexpr float c_sSlaiSafe = 3000.0f;
constexpr float c_sSlaiFollowStop = 900.0f, c_sSlaiFollowResume = 1500.0f;
// Preserve the original comfortable visit distance; visibility alone no longer starts flight.
constexpr float c_sSlaiVisitStop = 1500.0f, c_sSlaiVisitResume = 2100.0f;
constexpr float c_sSlaiVisitRange = 6000.0f;
constexpr float c_gSlaiVisual = 0.70710678f; // original Carmelita 90 degree horizontal FOV, not camera
constexpr int c_oidSlaiCarmThreat = 7727; // Carmelita's R2 gesture
bool FSlaiCarmGestureThreat(bool visible,int anim,float heading,float dx,float dy)
{
    if(!visible||anim!=c_oidSlaiCarmThreat)return false;
    const float t2=dx*dx+dy*dy;
    return t2<1.f || std::cos(heading)*dx+std::sin(heading)*dy>=c_gSlaiVisual*std::sqrt(t2);
}
constexpr float c_gSlaiFollowChance = 0.30f;
constexpr float c_gSlaiVisitChance = 0.30f;
// Guard projectile damage cadence, not the guard's native firing animation cadence.
constexpr float c_tSlaiGuardShotInterval = 2.5f;
constexpr float c_tSlaiGuardShotRecovery = 1.6f;
constexpr float c_tSlaiGuardShotGroundStun = 0.85f;
float SlaiGuardShotStun(float clipTime, bool airborne)
{
    return airborne ? clipTime : std::min(clipTime,c_tSlaiGuardShotGroundStun);
}
float SlaiGuardShotDeadline(float now,float hurtUntil)
{
    return std::max(now+c_tSlaiGuardShotInterval,hurtUntil+c_tSlaiGuardShotRecovery);
}
float RandomRange(float a, float b) { return std::uniform_real_distribution<float>(a,b)(s_rng); }
// A curiosity roll starts a bounded episode, never a per-frame lottery.
// A failed roll or a finished episode creates an independent exploration window.
bool StepSlaiCuriosity(LIVE& l,float now,bool eligible,bool tracking)
{
    if(l.followDeadline>0)
    {
        if(tracking&&now<l.followDeadline)return true;
        l.followDeadline=0;
        l.nextCuriosity=now+RandomRange(25,45);
        return false;
    }
    if(!eligible||now<l.nextCuriosity||now<l.socialCooldown)return false;
    if(RandomRange(0,1)<c_gSlaiFollowChance)
    {
        l.followDeadline=now+RandomRange(10,20);
        return true;
    }
    l.nextCuriosity=now+RandomRange(25,45);
    return false;
}
void ResetSocial(LIVE& l,float now,bool pause)
{
    l.following=false;l.followHolding=false;l.visiting=false;
    l.holdUntil=0;l.lastPlayerSeen=0;l.visitDeadline=0;l.visitEnd=0;
    l.nextVisit=now+RandomRange(60,120);
    l.socialCooldown=pause?now+RandomRange(10,20):0;
    l.followDeadline=0;l.nextCuriosity=pause?now+RandomRange(25,45):now;
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
void CancelSlaiBinoculars(LIVE& l)
{
    if(l.binocularPhase){l.binocularPhase=0;l.binocularUntil=0;l.restUntil=0;}
}
bool StepSlaiBinoculars(LIVE& l,float now)
{
    if(l.binocularPhase==1 && now>=l.binocularUntil){
        l.binocularPhase=2;
        l.binocularUntil=now+ClipTime(l.binocularOut,0.5f);
        RestartClip(l,l.binocularOut);
    }
    if(l.binocularPhase==2 && now>=l.binocularUntil){l.binocularPhase=0;l.binocularUntil=0;}
    return l.binocularPhase!=0;
}
void FaceSlai(LIVE& l, const glm::vec3& target, float dt)
{
    const glm::vec3 d = target - l.pos;
    if (d.x*d.x+d.y*d.y < 1.0f) return;
    const float wanted = std::atan2(d.y,d.x);
    l.heading += std::clamp(std::remainder(wanted-l.heading, 6.2831853f), -6.0f*dt, 6.0f*dt);
}
float Distance(const glm::vec3& a,const glm::vec3& b) { return glm::length(a-b); }
void RememberSlaiFailure(LIVE& l,const glm::vec3& pos,bool jump,float now)
{
    if(!std::isfinite(pos.x)||!std::isfinite(pos.y)||!std::isfinite(pos.z))return;
    auto& mem=l.failedGoals;
    mem.erase(std::remove_if(mem.begin(),mem.end(),[&](const SlaiFailedGoal& f){
        return f.until<=now||(f.jump==jump&&Distance(f.pos,pos)<75.f);
    }),mem.end());
    mem.push_back({pos,now+(jump?20.f:30.f),jump});
    if(mem.size()>12)mem.pop_front();
    if(FPrint())std::printf("Slai: remembered failed %s goal %.0f %.0f %.0f\n",jump?"jump":"walk",pos.x,pos.y,pos.z);
}
bool SlaiFailedGoalNear(const LIVE& l,const glm::vec3& q,bool jump,float now)
{
    for(const auto& f:l.failedGoals){
        if(f.jump!=jump||f.until<=now||Distance(q,f.pos)>90.f)continue;
        // Do not trap the actor inside a rejected walking area: allow moving out.
        if(jump||Distance(q,f.pos)<=Distance(l.pos,f.pos)+15.f)return true;
    }
    return false;
}
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
    const int count=std::clamp(int(std::ceil(dist/30.0f)),1,32);
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
        l.commandedGoal=desired;l.hasCommandedGoal=true;
        FaceSlai(l,desired,dt);
        v*=std::min(speed,len/std::max(dt,0.001f))/len;
    }
    else
    {
        v=glm::vec3(0);
        l.hasCommandedGoal=false;
        if(step->pxpFloor) { const auto old=VelAloPos(a); SetAloVelocityTo(a,glm::vec3(0,0,old.z)); }
    }
    // STEP's floor solver uses negative desired velocity in the heading frame (as GUARD does).
    // Keep vertical integration, floor contacts, collision response and angular braking native.
    step->nMotion=0; step->fNoContacts=0;
    SetStepPhysHeading(a,l.heading);
    // Native floor fitting is opt-in until tested against the JT collision mesh.
    // Reuse its cached heading frame instead of six terrain probes per frame.
    if(std::getenv("SLY2N_SLAIFLOORFIT") && step->pxpFloor && speed>0.f && len>10.f){
        const float now=g_clock.t;
        if(now>=l.nextFloorFit || std::abs(std::remainder(l.heading-l.floorFitHeading,6.2831853f))>0.15f){
            l.nextFloorFit=now+0.25f;l.floorFitHeading=l.heading;l.floorFitValid=false;
            const glm::vec3 offset=glm::transpose(step->matHeading)*(l.pos-PosLoWorld(a));
            const glm::vec3 probes[3]={offset+glm::vec3(-24,-20,0),offset+glm::vec3(36,-20,0),offset+glm::vec3(0,30,0)};
            bool valid=true;float low=1e20f,high=-1e20f;
            for(const auto& probe:probes){glm::vec3 q=PosLoWorld(a)+step->matHeading*probe;
                if(!FDropOnGround(&q,40,-45,false)||SlaiWetPoint(q)){valid=false;break;}
                low=std::min(low,q.z);high=std::max(high,q.z);}
            if(valid&&high-low>5.f&&high-low<50.f){
                FitStepToFloor(a,-45.f,40.f,3,probes);
                l.floorFrame=step->matHeading;l.floorFitValid=l.floorFrame[2].z>0.8f;
            }
        }
        if(l.floorFitValid){step->matHeading=l.floorFrame;
            const glm::vec3 n=l.floorFrame[2];v.z=-(n.x*v.x+n.y*v.y)/n.z;
            const float total=glm::length(v);if(total>speed)v*=speed/total;
        }else SetStepPhysHeading(a,l.heading);
    }else l.floorFitValid=false;
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
        if(SlaiFailedGoalNear(l,q,false,g_clock.t))continue;
        float score=glm::dot(v,forward)*length-std::abs(angle)*12;
        // Prefer a route with space on both sides, rather than scraping the wall.
        const glm::vec2 side(-v.y,v.x);
        for(float sign:{-1.f,1.f}){
            const glm::vec3 a=q+glm::vec3(0,0,65), b=a+glm::vec3(side*(sign*55.f),0);
            if(!SlaiClearLine(a,b))score-=45.f;
        }
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
            glm::vec3 ahead(0);if(!SamplePath(c,ns+dir*100,ahead)||glm::length(glm::vec2(ahead-q))<15||!GroundConnection(q,ahead)||SlaiFailedGoalNear(l,ahead,false,g_clock.t))continue;
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
        if(SlaiFailedGoalNear(l,to,true,now))continue;
        if(interest&&(away?1.f:-1.f)*(Distance(to,*interest)-Distance(l.pos,*interest))<45.f)continue;
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
bool SlaiRoofArc(const glm::vec3& start,const glm::vec3& vel,float g,float t,bool dry=false)
{
    if(!std::isfinite(t)||t<=0||t>2.5f)return false;
    const glm::vec2 v(vel);const float speed=glm::length(v);
    const glm::vec2 side=speed>1?glm::vec2(-v.y,v.x)/speed:glm::vec2(1,0);
    glm::vec3 last=start;
    const int n=std::clamp(int(std::ceil(t/0.06f)),8,44);
    for(int i=1;i<=n;++i){
        const float u=t*float(i)/n;
        const glm::vec3 q=start+vel*u-glm::vec3(0,0,0.5f*g*u*u);
        if(dry)for(float w:{0.f,-24.f,24.f})if(SlaiWetPoint(q+glm::vec3(side*w,0)))return false;
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
    return SlaiRoofArc(start,vel,g,flight,true);
}
// A directed local link, with a floor probe at both ends and a swept body arc.
// No scripted pads, forced positions or guessed landing heights.
struct SlaiVerticalPlan {
    glm::vec3 velocity{0}, apex{0};
    float flight=0, secondAt=0;
    bool waterCross=false;
};
bool SlaiVerticalFloor(const glm::vec3& q)
{
    if(SlaiWetPoint(q))return false;
    for(glm::vec2 offset:{glm::vec2(0),glm::vec2(28,0),glm::vec2(-28,0),glm::vec2(0,28),glm::vec2(0,-28)}){
        glm::vec3 p=q+glm::vec3(offset,0);
        if(!FDropOnGround(&p,35,-35,false)||std::abs(p.z-q.z)>22||SlaiWetPoint(p)||
           !SlaiClearLine(p+glm::vec3(0,0,12),p+glm::vec3(0,0,155)))return false;
    }
    return true;
}
// A water link requires a real intervening water volume, two dry banks and
// a physically reachable arc. A wet volume beneath a walkable bridge is not a link.
bool SlaiWaterBetween(const glm::vec3& from,const glm::vec3& to)
{
    const float span=glm::length(glm::vec2(to-from));
    if(span<65.f||span>480.f||std::abs(to.z-from.z)>160.f)return false;
    const int n=std::clamp(int(std::ceil(span/15.f)),5,32);
    for(int i=1;i<n;++i){
        const glm::vec3 q=glm::mix(from,to,float(i)/n);
        for(float down:{0.f,60.f,120.f,180.f})
            if(SlaiWetPoint(q-glm::vec3(0,0,down)))return true;
    }
    return false;
}
bool SlaiBankExit(const glm::vec3& to)
{
    for(int i=0;i<8;++i){
        const float a=i*0.78539816f;
        glm::vec3 q=to+glm::vec3(65.f*std::cos(a),65.f*std::sin(a),0);
        if(FDropOnGround(&q,35,-35,false)&&GroundConnection(to,q))return true;
    }
    return false;
}
bool PlanSlaiWaterCross(LIVE& l,const glm::vec3& from,const glm::vec3& to,SlaiVerticalPlan& plan)
{
    plan=SlaiVerticalPlan{};
    if(!SlaiWaterBetween(from,to)||GroundConnection(from,to)||!SlaiVerticalFloor(from)||!SlaiVerticalFloor(to)||!SlaiBankExit(to))return false;
    const SOST* physics=Psost(static_cast<ALO*>(l.plo));
    if(!physics||!std::isfinite(physics->dvGravity.z)||physics->dvGravity.z>=-1.f)return false;
    const float g=-physics->dvGravity.z;
    const glm::vec3 d=to-from;
    const float span=glm::length(glm::vec2(d));
    for(float margin:{105.f,145.f,190.f,230.f}){
        const float vz=std::sqrt(2.f*g*(std::max(0.f,d.z)+margin));
        const float t=(vz+std::sqrt(std::max(0.f,vz*vz-2.f*g*d.z)))/g;
        if(!std::isfinite(t)||t<0.2f||t>2.f||span/t>c_sSlaiRun)continue;
        const glm::vec3 v(d.x/t,d.y/t,vz);
        bool dry=true;
        const int n=std::clamp(int(std::ceil(t/0.035f)),12,60);
        const glm::vec2 side(-d.y/span,d.x/span);
        for(int i=0;i<=n&&dry;++i){
            const float u=t*float(i)/n;
            const glm::vec3 q=from+v*u-glm::vec3(0,0,0.5f*g*u*u);
            for(float w:{0.f,-24.f,24.f})if(SlaiWetPoint(q+glm::vec3(side*w,0))){dry=false;break;}
        }
        if(dry&&SlaiRoofArc(from,v,g,t)){plan={v,{},t,0,true};return true;}
    }
    return false;
}
bool LaunchSlaiWaterCross(LIVE& l,const glm::vec3& to,LO* path,float s,int dir,float now)
{
    STEP* st=PstepFromLo(l.plo);
    if(!st||!st->pxpFloor||!l.jumpClip||l.jumping||now<l.nextJump||now<l.hurtUntil||SlaiFailedGoalNear(l,to,true,now))return false;
    SlaiVerticalPlan plan;
    if(!PlanSlaiWaterCross(l,l.pos,to,plan))return false;
    l.verticalApproach=false;l.verticalFlight=true;l.roofFlight=false;l.roofSecondPending=false;
    l.waterCrossFlight=true;l.jumping=true;l.jumpStarted=now;l.jumpDeadline=now+plan.flight+1.f;
    l.jumpGoal=to;l.jumpPath=path;l.jumpS=s;l.jumpDir=dir;l.freeUntil=0;
    l.heading=std::atan2(to.y-l.pos.y,to.x-l.pos.x);SetStepPhysHeading(st->palo,l.heading);
    l.verticalState="jump-water-cross";l.state=l.verticalState;
    st->nMotion=-1;st->vecVel=glm::vec3(0);st->pxpFloor=nullptr;
    SetAloVelocityTo(st->palo,plan.velocity);WakeAlo(st->palo);RestartClip(l,l.jumpClip);
    std::printf("Slai: jump-water-cross to %.0f %.0f %.0f\n",to.x,to.y,to.z);
    return true;
}
bool PlanSlaiVertical(LIVE& l,const glm::vec3& from,const glm::vec3& to,SlaiVerticalPlan& plan)
{
    plan=SlaiVerticalPlan{};
    const glm::vec3 d=to-from;
    const float span=glm::length(glm::vec2(d));
    if(SlaiWaterBetween(from,to)&&!GroundConnection(from,to))return PlanSlaiWaterCross(l,from,to,plan);
    if(span<60||span>400||d.z<-550||d.z>600||!SlaiVerticalFloor(to)||GroundConnection(from,to))return false;
    const SOST* physics=Psost(static_cast<ALO*>(l.plo));
    if(!physics||!std::isfinite(physics->dvGravity.z)||physics->dvGravity.z>=-1)return false;
    const float g=-physics->dvGravity.z;
    if(d.z<=280){
        // Optional authored clip timing, validated with the actual JT gravity and arc.
        // Do not start K2PLAY itself: its guard event listener is not installed on JT.
        if(l.nativeJumpTiming && std::getenv("SLY2N_SLAIK2TIMING")){
            const float t=l.nativeJumpTiming->dt14;
            if(std::isfinite(t)&&t>=0.25f&&t<=2.f&&span/t<=c_sSlaiRun){
                const glm::vec3 vel(d.x/t,d.y/t,d.z/t+0.5f*g*t);
                if(vel.z>0.f&&vel.z-g*t<0.f&&vel.z*vel.z/(2.f*g)<=480.f&&SlaiRoofArc(from,vel,g,t,true)){
                    plan={vel,{},t,0};return true;
                }
            }
        }
        // Car/ledge-sized surfaces: a forward single leap first, with room to clear the lip.
        for(float margin:{105.f,165.f,230.f,300.f}){
            if(std::max(0.f,d.z)+margin>480.f)continue;
            const float vz=std::sqrt(2*g*(std::max(0.f,d.z)+margin));
            const float t=(vz+std::sqrt(std::max(0.f,vz*vz-2*g*d.z)))/g;
            if(!std::isfinite(t)||t<0.2f||t>2.f||span/t>c_sSlaiRun)continue;
            const glm::vec3 v(d.x/t,d.y/t,vz);
            if(SlaiRoofArc(from,v,g,t,true)){plan={v,{},t,0};return true;}
        }
        if(d.z<=145.f)return false;
    }
    // During escape, do not stop for a vertical roof-climbing manoeuvre.
    if(l.fleeing||!l.doubleJumpClip)return false;
    // Ascend beside the wall; the second impulse is aimed at the same landing.
    const float rise=340.f,vz=std::sqrt(2*g*rise),t=vz/g;
    const glm::vec3 v(0,0,vz), apex=from+glm::vec3(0,0,rise);
    glm::vec3 second(0);float secondT=0;
    if(!SlaiRoofArc(from,v,g,t,true)||!SlaiRoofSecond(apex,to,g,second,secondT))return false;
    plan={v,apex,t+secondT,t};return true;
}
bool StepSlaiVertical(LIVE& l,const glm::vec3& intent,bool away,float now,float dt)
{
    STEP* st=PstepFromLo(l.plo);
    if(!st||!st->pxpFloor||!l.jumpClip||SlaiWetPoint(l.pos)) {l.verticalApproach=false;return false;}
    if(l.verticalApproach&&(now>=l.verticalUntil||away!=l.verticalAway||(!away&&Distance(intent,l.verticalIntent)>450))){
        if(now>=l.verticalUntil)RememberSlaiFailure(l,l.verticalTo,true,now);
        l.verticalApproach=false;
    }
    if(!l.verticalApproach){
        glm::vec3 ground=l.pos;
        if(!FDropOnGround(&ground,30,-35,false)||std::abs(ground.z-l.pos.z)>30)return false;
        if(now<l.nextVertical||now<l.nextJump)return false;
        l.nextVertical=now+(away?4.f:2.f);
        glm::vec2 f(intent-l.pos);if(away)f=-f;
        if(glm::length(f)<1)f=glm::vec2(std::cos(l.heading),std::sin(l.heading));else f=glm::normalize(f);
        const glm::vec2 side(-f.y,f.x);
        std::vector<glm::vec3> starts{l.pos};
        for(glm::vec2 off:{-f*90.f,side*90.f,-side*90.f}){
            if(away)break; // escape never walks to a distant takeoff point
            glm::vec3 q=l.pos+glm::vec3(off,0);
            if(FDropOnGround(&q,40,-50,false)&&std::abs(q.z-l.pos.z)<35&&SlaiVerticalFloor(q)&&GroundConnection(l.pos,q))starts.push_back(q);
        }
        struct Candidate {glm::vec3 from,to;float score;};
        std::vector<Candidate> candidates;
        // Sample actual collision floors in distinct height bands, including downward.
        for(const auto& from:starts)for(float angle:{0.f,.55f,-.55f,1.1f,-1.1f})
        for(float r:{80.f,125.f,190.f,280.f,365.f,460.f}){
            if(away&&(std::abs(angle)>1.1f||r==190.f||r==365.f))continue;
            const glm::vec2 dir=f*std::cos(angle)+side*std::sin(angle);
            for(glm::vec2 band:{glm::vec2(280,-55),glm::vec2(600,280),glm::vec2(-55,-550)}){
                if(away&&band.x==600.f)continue;
                glm::vec3 q=from+glm::vec3(dir*r,0);
                if(!FDropOnGround(&q,band.x,band.y,false)||SlaiWetPoint(q)||
                    (now<l.blockedUntil&&Distance(q,l.blockedPos)<90)||SlaiFailedGoalNear(l,q,true,now))continue;
                if(r>400.f&&!SlaiWaterBetween(from,q))continue;
                if(away&&(q.z-from.z>220.f||q.z-from.z<-150.f))continue;
                const float gain=(away?-1.f:1.f)*(Distance(l.pos,intent)-Distance(q,intent));
                if(gain<45)continue; // Every jump must make progress towards its current purpose.
                // Do not fill the limited arc-test budget with ordinary walkable endpoints.
                if(std::abs(q.z-from.z)<=65.f&&GroundConnection(from,q))continue;
                const float score=gain-0.22f*Distance(l.pos,from)-0.12f*r-(away?0.65f:0.18f)*std::abs(q.z-from.z);
                candidates.push_back({from,q,score});
            }
        }
        std::sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){return a.score>b.score;});
        int checked=0;
        for(const auto& c:candidates){
            if(++checked>(away?5:12))break;
            SlaiVerticalPlan test;
            if(!PlanSlaiVertical(l,c.from,c.to,test))continue;
            l.verticalFrom=c.from;l.verticalTo=c.to;l.verticalIntent=intent;l.verticalAway=away;
            l.verticalApproach=true;l.verticalUntil=now+1.5f;
            std::printf("Slai: vertical plan from %.0f %.0f %.0f to %.0f %.0f %.0f\n",c.from.x,c.from.y,c.from.z,c.to.x,c.to.y,c.to.z);
            break;
        }
        if(!l.verticalApproach)return false;
    }
    const float distance=glm::length(glm::vec2(l.verticalFrom-l.pos));
    if(distance>18){
        if(!GroundConnection(l.pos,l.verticalFrom)){RememberSlaiFailure(l,l.verticalTo,true,now);l.verticalApproach=false;return false;}
        SlaiDrive(l,l.verticalFrom,std::min(c_sSlaiWalk,distance*5.f),dt);PlayClip(l,l.walk);
        l.state="jump-approach";return true;
    }
    SlaiVerticalPlan plan;
    l.verticalApproach=false;
    if(!PlanSlaiVertical(l,l.pos,l.verticalTo,plan)){RememberSlaiFailure(l,l.verticalTo,true,now);l.nextVertical=now+0.8f;return false;}
    l.verticalFlight=true;l.roofFlight=plan.secondAt>0;l.roofSecondPending=l.roofFlight;
    l.roofSecondAt=now+plan.secondAt;l.roofApex=plan.apex;
    l.jumping=true;l.jumpStarted=now;l.jumpDeadline=now+plan.flight+1.f;
    l.jumpGoal=l.verticalTo;l.jumpPath=nullptr;l.freeUntil=0;
    l.heading=std::atan2(l.jumpGoal.y-l.pos.y,l.jumpGoal.x-l.pos.x);SetStepPhysHeading(st->palo,l.heading);
    l.waterCrossFlight=plan.waterCross;
    l.verticalState=l.waterCrossFlight?"jump-water-cross":l.jumpGoal.z-l.pos.z>45?"jump-up":l.jumpGoal.z-l.pos.z<-45?"jump-down":"jump-across";
    st->nMotion=-1;st->vecVel=glm::vec3(0);st->pxpFloor=nullptr;
    SetAloVelocityTo(st->palo,plan.velocity);WakeAlo(st->palo);RestartClip(l,l.jumpClip);
    l.state=l.verticalState;
    std::printf("Slai: %s toward %.0f %.0f %.0f\n",l.state,l.jumpGoal.x,l.jumpGoal.y,l.jumpGoal.z);
    return true;
}
// Correct horizontal drift toward a committed landing, preserving native vertical physics.
// Do not steer the intentional first vertical phase of a roof double jump.
void SteerSlaiAir(LIVE& l,STEP* st,float now)
{
    if(!l.verticalFlight||l.roofSecondPending||st->pxpFloor||now<l.hurtUntil||now>=l.jumpDeadline)return;
    const SOST* physics=Psost(st->palo);
    if(!physics||!std::isfinite(physics->dvGravity.z)||physics->dvGravity.z>=-1.f)return;
    const float g=-physics->dvGravity.z;
    const glm::vec3 current=VelAloPos(st->palo), delta=l.jumpGoal-l.pos;
    if(!std::isfinite(current.x)||!std::isfinite(current.y)||!std::isfinite(current.z))return;
    const float disc=current.z*current.z-2.f*g*delta.z;
    if(disc<0.f||!std::isfinite(disc))return; // target is above the remaining apex
    const float remain=(current.z+std::sqrt(disc))/g;
    if(remain<=0.035f)return;
    const float dt=std::clamp(g_clock.dt,0.f,0.05f);
    if(dt<=0.f)return;
    const glm::vec2 to(delta), old(current);
    glm::vec2 want=to/std::max(remain,0.12f);
    const float speed=glm::length(want);
    if(speed>c_sSlaiRun)want*=c_sSlaiRun/speed;
    glm::vec2 adjust=want-old;
    const float change=glm::length(adjust), maxChange=2200.f*dt;
    if(change<6.f)return;
    if(change>maxChange)adjust*=maxChange/change;
    const glm::vec2 next=old+adjust;
    const glm::vec3 velocity(next,current.z);
    // Check the short correction before applying it; do not inject movement through a wall.
    // The same body-width/height rays as the launch planner are used here.
    const float horizon=std::min(remain,0.12f);
    const glm::vec3 predicted=l.pos+velocity*horizon-glm::vec3(0,0,0.5f*g*horizon*horizon);
    const float nextSpeed=glm::length(next);
    const glm::vec2 side=nextSpeed>1.f?glm::vec2(-next.y,next.x)/nextSpeed:glm::vec2(1,0);
    // Short swept probes rather than re-running an entire ballistic planner every frame.
    for(float w:{0.f,-24.f,24.f}){
        const glm::vec3 off(side*w,0);
        if(SlaiWetPoint(predicted+off))return;
        for(float height:{18.f,75.f,145.f}){
            const glm::vec3 ray=off+glm::vec3(0,0,height);
            if(!SlaiClearLine(l.pos+ray,predicted+ray))return;
        }
    }
    SetAloVelocityTo(st->palo,velocity);WakeAlo(st->palo);
    if(glm::length(to)>15.f){FaceSlai(l,l.jumpGoal,dt);SetStepPhysHeading(st->palo,l.heading);}
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
            SlaiVerticalFloor(l.jumpGoal)&&SlaiRoofSecond(l.pos,l.jumpGoal,-physics->dvGravity.z,vel,t)){
            st->nMotion=-1;st->vecVel=glm::vec3(0);st->pxpFloor=nullptr;
            SetAloVelocityTo(st->palo,vel);WakeAlo(st->palo);RestartClip(l,l.doubleJumpClip);
            l.jumpDeadline=now+t+0.8f;
            std::printf("Slai: roof second jump\n");return true;
        }
        l.jumpDeadline=now+1.5f;
        RememberSlaiFailure(l,l.jumpGoal,true,now);
        std::printf("Slai: roof second jump cancelled\n");
    }
    glm::vec3 q=l.pos;
    const bool landed=!l.roofSecondPending&&now-l.jumpStarted>0.18f&&vz<100&&st->pxpFloor&&FDropOnGround(&q,30,-35,false)&&std::abs(q.z-l.pos.z)<30&&!SlaiWetPoint(q);
    if(!landed&&(now<l.jumpDeadline||(l.verticalFlight&&now<l.jumpDeadline+1.5f&&!st->pxpFloor&&l.pos.z>l.jumpGoal.z-150.f)))
    {
        st->nMotion=-1;st->vecVel=glm::vec3(0);
        SteerSlaiAir(l,st,now);
        return true; // horizontal guidance only; never repeat the upward impulse
    }
    const bool vertical=l.verticalFlight, crossing=l.waterCrossFlight;
    l.jumpRecoveryAirborne=!landed&&!st->pxpFloor;
    l.waterCrossFlight=false;
    l.verticalFlight=false;l.jumping=false;l.roofSecondPending=false;l.roofFlight=false;st->nMotion=0;st->vecVel=glm::vec3(0);
    l.lastProgress=now;l.nextProgressCheck=now+0.5f;l.progressPos=l.pos;l.nextJump=now+(vertical?1.5f:3.f);
    if(landed)
    {
        const bool arrived=crossing
            ? glm::length(glm::vec2(q-l.jumpGoal))<=75.f && std::abs(q.z-l.jumpGoal.z)<=35.f
              && SlaiVerticalFloor(q) && GroundConnection(q,l.jumpGoal)
            : glm::length(glm::vec2(q-l.jumpGoal))<=85.f && std::abs(q.z-l.jumpGoal.z)<=35.f;
        if(!arrived&&!l.waterJump)RememberSlaiFailure(l,l.jumpGoal,true,now);
        if(arrived&&l.jumpPath){RememberPath(l,l.path);l.path=l.jumpPath;l.s=l.jumpS;l.dir=l.jumpDir;}
        else {float s=0;LO* path=FindPatrol(psw,l.pos,s,nullptr,150);glm::vec3 entry(0);
            if(path&&SamplePath(PcrvFromLo(path),s,entry)&&GroundConnection(l.pos,entry)){
                RememberPath(l,l.path);l.path=path;l.s=s;
            }else {l.path=nullptr;l.nextSearch=now;}}
        l.routeSince=now; // last dry point is updated only by the stable-floor sampler
        if(vertical){
            l.freeUntil=0;l.nextLocal=now;l.nextVertical=now+(arrived?1.5f:5.f);
            if(!arrived){l.nextJump=now+5.f;l.blockedPos=l.jumpGoal;l.blockedUntil=now+8;}
            l.state="landing";
        }
        std::printf("Slai: %s landed (%s)\n",crossing?"water crossing":"jump",arrived?"target":"other floor");
    }
    else {if(!l.waterJump)RememberSlaiFailure(l,l.jumpGoal,true,now);l.path=nullptr;l.nextSearch=now;if(vertical){l.blockedPos=l.jumpGoal;l.blockedUntil=now+8;l.nextVertical=now+5.f;l.nextJump=now+5.f;}std::printf("Slai: jump timeout\n");}
    l.jumpPath=nullptr;
    if(vertical&&landed){l.hasCommandedGoal=false;} // resume ground decisions immediately
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
    if(!wet && !l.waterCrossFlight && actor && actor->ploWater && actor->ploWater->fInWorld){
        if(OXA* ox=Poxa(static_cast<ALO*>(actor->ploWater)))
            wet=l.pos.x>=ox->posMin.x&&l.pos.x<=ox->posMax.x&&l.pos.y>=ox->posMin.y&&l.pos.y<=ox->posMax.y&&l.pos.z<ox->posMax.z-5&&l.pos.z+75>ox->posMin.z;
    }
    if(l.waterJump){
        if(StepSlaiJump(psw,l,st,now)){l.state="water-jump";return true;}
        l.waterJump=false;
        if(!wet){l.waterSince=0;return false;}
    }
    if(!wet){l.waterSince=0;return false;}
    if(l.waterSince==0){if(l.waterCrossFlight){RememberSlaiFailure(l,l.jumpGoal,true,now);l.waterCrossFlight=false;std::printf("Slai: water crossing failed\n");}l.waterSince=now;l.verticalApproach=false;l.verticalFlight=false;l.jumping=false;l.jumpRecoveryAirborne=false;l.roofSecondPending=false;l.roofFlight=false;l.jumpPath=nullptr;ResetSlaiNav(l);l.path=nullptr;l.freeUntil=0;}
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
    l.state="water-recover";SlaiDrive(l,l.pos,0,dt);if(l.waterClip && now>=l.hurtUntil)PlayClip(l,l.waterClip);return true;
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
                if(duplicate||SlaiFailedGoalNear(l,q,false,now)||!GroundConnection(from,q))continue;
                l.navNodes.push_back({q,idx});
            }
        }
        int best=-1;float value=0;const float base=Distance(l.navNodes[0].pos,threat);
        for(size_t i=1;i<l.navNodes.size();++i){const auto& q=l.navNodes[i].pos;const float gain=Distance(q,threat)-base;const float score=gain+0.12f*Distance(q,l.navNodes[0].pos);if(score>value){best=int(i);value=score;}}
        if((best>0&&value>90)||l.navCursor>=l.navNodes.size()||l.navNodes.size()>=56||now-l.navStarted>=0.35f){
            if(best<0||value<55){ResetSlaiNav(l);l.navCooldown=now+4;return false;}
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
        l.pickupSkipPos=l.pickupGoal;l.pickupSkipUntil=now+30;l.pickupUntil=0;l.pickupStarted=0;
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

// Join authored guard patrols via bounded, physically checked intermediate waypoints.
// No GUARD cast of Slai, no script execution, no movement by teleportation.
void ResetSlaiPatrolPlan(LIVE& l)
{
    l.patrolNodes.clear(); l.patrolEntries.clear(); l.patrolRoute.clear();
    l.patrolCursor=0; l.patrolTarget=nullptr;
}
bool SlaiPatrolSample(const CRV* c,float s,glm::vec3& q)
{
    if(!c||c->as.size()<2)return false;
    glm::vec3 tangent(0),raw(0);
    EvalCrvS(c,BoundS(c,s),&raw,&tangent);
    if(!std::isfinite(raw.x)||!std::isfinite(raw.y)||!std::isfinite(raw.z))return false;
    q=raw;
    // Preserve the authored floor level: don't project a roof route onto a street far below.
    return FDropOnGround(&q,100,-100,false)&&std::abs(q.z-raw.z)<=100&&!SlaiWetPoint(q);
}
bool TrySlaiPatrolWater(LIVE& l,const CRV* c,float now)
{
    STEP* st=PstepFromLo(l.plo);
    if(!c||!l.path||!st||!st->pxpFloor||!l.jumpClip||now<l.nextJump||now<l.nextWaterCrossProbe)return false;
    l.nextWaterCrossProbe=now+0.8f;
    glm::vec3 near(0);
    if(SlaiPatrolSample(c,l.s+l.dir*85.f,near)&&Distance(l.pos,near)>15.f&&GroundConnection(l.pos,near))return false;
    for(float offset:{130.f,190.f,250.f,310.f,370.f,430.f,480.f}){
        const float at=BoundS(c,l.s+l.dir*offset);
        glm::vec3 q(0),ahead(0);
        if(!SlaiPatrolSample(c,at,q)||!SlaiPatrolSample(c,at+l.dir*65.f,ahead)||Distance(q,ahead)<15.f||!GroundConnection(q,ahead))continue;
        if(LaunchSlaiWaterCross(l,q,l.path,at,l.dir,now))return true;
    }
    return false;
}
bool StepSlaiPatrolPlan(SW* psw,LIVE& l,bool request,bool stuck,float now,float dt)
{
    auto fail=[&](float delay){ResetSlaiPatrolPlan(l);l.patrolRetry=now+std::max(delay,8.0f);return false;};
    if(l.patrolTarget){
        if(stuck&&now-l.patrolStarted>1.3f){
            if(!l.patrolRoute.empty())RememberSlaiFailure(l,l.patrolRoute.front(),false,now);
            return fail(3.f);
        }
        while(!l.patrolRoute.empty()&&Distance(l.pos,l.patrolRoute.front())<30.f)l.patrolRoute.pop_front();
        if(l.patrolRoute.empty()){
            const CRV* c=PcrvFromLo(l.patrolTarget);glm::vec3 q(0);
            if(!SlaiPatrolSample(c,l.patrolTargetS,q)||Distance(l.pos,q)>55.f||!GroundConnection(l.pos,q))return fail(3.f);
            if(l.path!=l.patrolTarget)RememberPath(l,l.path);
            l.path=l.patrolTarget;l.s=l.patrolTargetS;l.dir=l.patrolTargetDir;
            l.routeSince=now;l.nextSwitch=now+6.f;l.nextDirection=now+2.f;
            l.freeUntil=0;l.lastProgress=now;l.progressPos=l.pos;l.nextProgressCheck=now+0.5f;
            std::printf("Slai: connected to native patrol %d\n",l.path->oid);
            ResetSlaiPatrolPlan(l);l.patrolRetry=now+10.f;
            SlaiDrive(l,l.pos,0,dt);PlaySlai(l,false);l.state="patrol-joined";return true;
        }
        const glm::vec3 q=l.patrolRoute.front();
        if(!GroundConnection(l.pos,q)){
            if(l.patrolRoute.size()==1&&SlaiWaterBetween(l.pos,q)){
                if(now<l.nextJump){SlaiDrive(l,l.pos,0,dt);PlaySlai(l,false);l.lastProgress=now;return true;}
                if(LaunchSlaiWaterCross(l,q,l.patrolTarget,l.patrolTargetS,l.patrolTargetDir,now)){ResetSlaiPatrolPlan(l);l.patrolRetry=now+8.f;return true;}
                RememberSlaiFailure(l,q,true,now);
            }else RememberSlaiFailure(l,q,false,now);
            return fail(3.f);
        }
        SlaiDrive(l,q,c_sSlaiWalk,dt);PlayClip(l,l.walk);l.state="patrol-connect";return true;
    }
    if(l.patrolNodes.empty()){
        if(!request||now<l.patrolRetry)return false;
        STEP* st=PstepFromLo(l.plo);if(!st||!st->pxpFloor)return false;
        // Gather nearby points from genuine patrol curves, including points beyond their nearest point.
        std::vector<SlaiPatrolEntry> rawEntries;
        for(LO* path:PatrolPaths(psw)){
            const CRV* c=PcrvFromLo(path);if(!c||c->as.size()<2)continue;
            glm::vec3 raw(0);float ns=0;NearestCrvAll(c,l.pos,&raw,nullptr,nullptr,&ns);
            if(!std::isfinite(ns)||Distance(l.pos,raw)>950.f)continue;
            const bool recent=std::find(l.recentPaths.begin(),l.recentPaths.end(),path)!=l.recentPaths.end();
            for(float offset:{0.f,180.f,-180.f,360.f,-360.f}){
                const float at=BoundS(c,ns+offset);glm::vec3 p(0),t(0);EvalCrvS(c,at,&p,&t);
                const float d=Distance(l.pos,p);
                if(!std::isfinite(d)||d<45.f||d>850.f||std::abs(p.z-l.pos.z)>160.f)continue;
                if(std::any_of(rawEntries.begin(),rawEntries.end(),[&](const SlaiPatrolEntry& e){return e.path==path&&std::abs(e.s-at)<35.f;}))continue;
                rawEntries.push_back({path,at,1,p,d+(path==l.path?140.f:0.f)+(recent?100.f:0.f)});
            }
        }
        std::sort(rawEntries.begin(),rawEntries.end(),[](const SlaiPatrolEntry& a,const SlaiPatrolEntry& b){return a.score<b.score;});
        int checked=0;
        for(auto e:rawEntries){
            if(++checked>18||l.patrolEntries.size()>=10)break;
            const CRV* c=PcrvFromLo(e.path);
            if(!SlaiPatrolSample(c,e.s,e.pos)||SlaiFailedGoalNear(l,e.pos,false,now))continue;
            bool exit=false;
            for(int dir:{1,-1}){glm::vec3 ahead(0);
                if(SlaiPatrolSample(c,e.s+dir*65.f,ahead)&&Distance(e.pos,ahead)>15.f&&GroundConnection(e.pos,ahead)&&!SlaiFailedGoalNear(l,ahead,false,now)){
                    e.dir=dir;exit=true;break;}}
            if(exit)l.patrolEntries.push_back(e);
        }
        if(l.patrolEntries.empty())return fail(4.f);
        l.patrolNodes.push_back({l.pos,-1});l.patrolCursor=0;l.patrolStarted=now;l.patrolNext=now;
    }
    if(now-l.patrolStarted>=0.9f||Distance(l.pos,l.patrolNodes.front().pos)>85.f)return fail(3.f);
    if(now>=l.patrolNext){
        l.patrolNext=now+0.08f;
        // Test one node per slice, using short links and a fixed maximum graph size.
        if(l.patrolCursor>=l.patrolNodes.size())return fail(3.f);
        const int idx=int(l.patrolCursor++);const glm::vec3 from=l.patrolNodes[idx].pos;
        // Try every short walking entry at this node before considering a water link.
        for(int pass=0;pass<2;++pass)for(const auto& e:l.patrolEntries){
            const float span=Distance(from,e.pos);
            if(span>500.f)continue;
            if(pass==0){
                if(span>185.f||!GroundConnection(from,e.pos))continue;
            }else{
                SlaiVerticalPlan crossing;
                if(SlaiFailedGoalNear(l,e.pos,true,now)||!PlanSlaiWaterCross(l,from,e.pos,crossing))continue;
            }
            l.patrolTarget=e.path;l.patrolTargetS=e.s;l.patrolTargetDir=e.dir;
            l.patrolRoute.push_front(e.pos);
            for(int i=idx;i>0;i=l.patrolNodes[i].parent)l.patrolRoute.push_front(l.patrolNodes[i].pos);
            l.patrolNodes.clear();l.patrolEntries.clear();l.patrolStarted=now;
            l.lastProgress=now;l.progressPos=l.pos;l.nextProgressCheck=now+0.5f;
            std::printf("Slai: native patrol connector %d (%zu waypoints)\n",e.path->oid,l.patrolRoute.size());
            return StepSlaiPatrolPlan(psw,l,false,false,now,dt);
        }
        for(int i=0;i<8&&l.patrolNodes.size()<56;++i){
            const float angle=i*0.78539816f;glm::vec3 q=from+glm::vec3(140.f*std::cos(angle),140.f*std::sin(angle),0);
            if(Distance(q,l.patrolNodes.front().pos)>800.f||!FDropOnGround(&q,50,-65,false)||SlaiWetPoint(q))continue;
            if(std::any_of(l.patrolNodes.begin(),l.patrolNodes.end(),[&](const SlaiNavNode& n){return Distance(n.pos,q)<75.f;}))continue;
            if(SlaiFailedGoalNear(l,q,false,now)||!GroundConnection(from,q))continue;
            l.patrolNodes.push_back({q,idx});
        }
    }
    SlaiDrive(l,l.pos,0,dt);PlaySlai(l,false);l.lastProgress=now;l.progressPos=l.pos;l.state="patrol-plan";return true;
}

// Reuse the world's original navmesh (NAVREGION) without pretending a JT is a GUARD.
// Physical validation remains mandatory: a guard mesh may include geometry Slai cannot use.
void ResetSlaiMesh(LIVE& l)
{
    l.meshRoute.clear();l.meshJump=false;l.meshMode=-1;l.meshUntil=0;
}
bool SlaiMeshFloor(const glm::vec3& raw,glm::vec3& q)
{
    q=raw;
    return std::isfinite(q.x)&&std::isfinite(q.y)&&std::isfinite(q.z)&&
        FDropOnGround(&q,65,-75,false)&&std::abs(q.z-raw.z)<=75.f&&!SlaiWetPoint(q);
}
bool SlaiMeshLeg(LIVE& l,NAVREGION* reg,const glm::vec3& from,const glm::vec3& to)
{
    if(SlaiFailedGoalNear(l,to,false,g_clock.t)||!GroundConnection(from,to))return false;
    // A mesh polygon is authoritative when both ends lie outside it. Permit leaving
    // a polygon if a dynamic object appeared around Slai, but never enter another.
    if(auto* poly=PpolyOnSegment(reg,from,to))
        if(!FPolyContains(poly,from)||FPolyContains(poly,to))return false;
    return true;
}
bool SlaiMeshPath(LIVE& l,NAVREGION* reg,const glm::vec3& target,std::deque<glm::vec3>& route)
{
    route.clear();if(!reg)return false;
    glm::vec4 pts[48];
    if(Distance(l.pos,target)<=260.f&&SlaiMeshLeg(l,reg,l.pos,target)&&!FNavBlocked(reg,l.pos,target,0)){
        route.push_back(target);return true;
    }
    const int count=CposNavPath(reg,l.pos,target,48,pts);
    if(count<2)return false;
    glm::vec3 last=l.pos;
    for(int i=1;i<count;++i){
        glm::vec3 q(0);if(!SlaiMeshFloor(glm::vec3(pts[i]),q))return false;
        if(Distance(last,q)<15.f)continue;
        // Split long mesh edges: avoid undersampling ramps or intervening obstacles.
        const int pieces=std::max(1,int(std::ceil(Distance(last,q)/260.f)));
        if(pieces>32||route.size()+size_t(pieces)>96)return false;
        const glm::vec3 start=last;
        for(int j=1;j<=pieces;++j){
            glm::vec3 p(0);if(!SlaiMeshFloor(glm::mix(start,q,float(j)/pieces),p))return false;
            if(!SlaiMeshLeg(l,reg,last,p)){
                // Native obstacle polygon: test a few outward corners, not an unbounded grid.
                NAVPOLY* poly=PpolyOnSegment(reg,last,p);
                if(!poly||poly->aposWorld.empty())return false;
                const glm::vec2 center((poly->posMin+poly->posMax)*0.5f);
                struct Candidate{glm::vec3 q;float cost;};std::vector<Candidate> candidates;
                for(const auto& v:poly->aposWorld){
                    glm::vec2 dir=glm::vec2(v)-center;float n=glm::length(dir);if(n<1.f)continue;
                    glm::vec3 q=glm::vec3(v)+glm::vec3(dir*(48.f/n),0);q.z=last.z;
                    candidates.push_back({q,Distance(last,q)+Distance(q,p)});
                }
                std::sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){return a.cost<b.cost;});
                bool found=false;int checked=0;
                for(const auto& c:candidates){if(++checked>6)break;glm::vec3 q;
                    if(!SlaiMeshFloor(c.q,q)||PregionAt(q,false)!=reg||FNavBlocked(reg,last,q,0)||FNavBlocked(reg,q,p,0))continue;
                    if(!SlaiMeshLeg(l,reg,last,q)||!SlaiMeshLeg(l,reg,q,p))continue;
                    if(route.size()>=95)return false;route.push_back(q);found=true;break;}
                if(!found)return false;
            }
            route.push_back(p);last=p;
        }
    }
    // CposNavPath can truncate or project a target onto a different floor.
    if(route.empty()||Distance(last,target)>95.f){route.clear();return false;}
    return true;
}
// Use simple, explicitly authored walking or jump links. Multi-record links
// can encode pipes / scripted actions and require a dedicated adapter.
bool SlaiMeshLink(LIVE& l,NAVREGION* reg,const glm::vec3& intent,bool away,
                  std::deque<glm::vec3>& route,glm::vec3& landing,bool& jump)
{
    jump=false;if(!reg)return false;
    float best=-1e20f;bool found=false;
    int tested=0;
    for(NAVLINK* link:AplinkWorld()){
        if(!link||!link->fResolved||(link->kind!=0&&link->kind!=1)||link->pregionBegin!=reg||link->agoal.size()!=2)continue;
        const bool isJump=link->kind==1;
        if(isJump&&g_clock.t<l.nextJump)continue;
        if(link->agoal[0].plo||link->agoal[1].plo)continue;
        glm::vec3 from(0),to(0);
        if(!SlaiMeshFloor(link->posBegin,from)||!SlaiMeshFloor(link->posEnd,to)||Distance(l.pos,from)>550.f||
           SlaiFailedGoalNear(l,to,isJump,g_clock.t))continue;
        const float gain=(away?-1.f:1.f)*(Distance(l.pos,intent)-Distance(to,intent));
        if(gain<40.f)continue;
        if(++tested>6)break;
        if(isJump){SlaiVerticalPlan plan;if(!PlanSlaiVertical(l,from,to,plan))continue;}
        else if(!GroundConnection(from,to))continue;
        std::deque<glm::vec3> candidate;
        if(Distance(l.pos,from)>18.f){
            if(GroundConnection(l.pos,from))candidate.push_back(from);
            else if(!SlaiMeshPath(l,reg,from,candidate))continue;
        }
        // Store the exact validated takeoff as the final waypoint, not just its projected triangle.
        if(!candidate.empty()&&Distance(candidate.back(),from)>12.f){
            if(!GroundConnection(candidate.back(),from))continue;
            candidate.push_back(from);
        }
        if(!isJump)candidate.push_back(to);
        const float score=gain-0.35f*Distance(l.pos,from)-(isJump?55.f:0.f);
        if(score>best){best=score;route=std::move(candidate);landing=to;found=true;jump=isJump;}
    }
    return found;
}
bool StepSlaiMesh(LIVE& l,const glm::vec3& intent,bool interested,bool away,bool stuck,bool request,float speed,float now,float dt)
{
    const int mode=away?2:interested?1:0;
    if(l.meshMode!=mode||l.meshGeneration!=NGenNav()||l.meshSerial!=NNavLinkSerial()||now>=l.meshUntil||
       (interested&&!away&&Distance(intent,l.meshThreat)>500.f))ResetSlaiMesh(l);
    auto reject=[&]{ResetSlaiMesh(l);l.meshRetry=now+2.f;return false;};
    if(l.meshMode>=0){
        if(stuck){if(!l.meshRoute.empty())RememberSlaiFailure(l,l.meshRoute.front(),false,now);return reject();}
        while(!l.meshRoute.empty()&&Distance(l.pos,l.meshRoute.front())<(l.meshJump&&l.meshRoute.size()==1?18.f:30.f)){l.meshRoute.pop_front();l.meshCheck=0;}
        if(!l.meshRoute.empty()){
            const auto q=l.meshRoute.front();
            if(now>=l.meshCheck){
                l.meshCheck=now+0.18f;
                NAVREGION* current=PregionAt(l.pos,false);
                const bool clear=current&&PregionAt(q,false)==current?SlaiMeshLeg(l,current,l.pos,q):GroundConnection(l.pos,q);
                if(!clear){RememberSlaiFailure(l,q,false,now);return reject();}
            }
            SlaiDrive(l,q,speed,dt);PlayClip(l,away&&l.run?l.run:l.walk);l.state="mesh-walk";return true;
        }
        if(l.meshJump){
            const auto to=l.meshLanding;ResetSlaiMesh(l);
            if(now<l.nextJump)return reject();
            // Revalidate with the existing physical jump driver at the actual launch position.
            l.verticalFrom=l.pos;l.verticalTo=to;l.verticalIntent=intent;l.verticalAway=away;
            l.verticalApproach=true;l.verticalUntil=now+1.f;
            if(StepSlaiVertical(l,intent,away,now,dt)){l.meshRetry=now+2.f;return true;}
            RememberSlaiFailure(l,to,true,now);return reject();
        }
        return reject();
    }
    if(!request||now<l.meshRetry)return false;
    l.meshRetry=now+2.f;
    STEP* st=PstepFromLo(l.plo);if(!st||!st->pxpFloor)return false;
    NAVREGION* reg=PregionAt(l.pos,false);if(!reg)return false;
    // First try a path in the same region. For escape, favour endpoints away from the player,
    // but allow the intermediate route to go around an obstruction in either direction.
    glm::vec2 dir(intent-l.pos);if(away)dir=-dir;
    if(glm::length(dir)<1.f)dir=glm::vec2(std::cos(l.heading),std::sin(l.heading));else dir=glm::normalize(dir);
    const glm::vec2 side(-dir.y,dir.x);
    bool found=false;
    // A target on another authored floor deserves a real link before random local steps.
    if(!away&&std::abs(intent.z-l.pos.z)>100.f&&PregionAt(intent,false)!=reg){
        found=SlaiMeshLink(l,reg,intent,false,l.meshRoute,l.meshLanding,l.meshJump);
    }
    for(float angle:{0.f,0.8f,-0.8f,1.55f,-1.55f}){
        if(found)break;
        const glm::vec2 f=dir*std::cos(angle)+side*std::sin(angle);
        glm::vec3 raw=l.pos+glm::vec3(f*360.f,0),q(0),floor(0);
        if(!away&&Distance(l.pos,intent)<360.f&&angle==0.f)raw=intent;
        if(!FKeepOnNav(reg,raw,&q)||!SlaiMeshFloor(q,floor)||Distance(l.pos,floor)<80.f||
           PregionAt(floor,false)!=reg)continue;
        if(away&&Distance(floor,intent)<Distance(l.pos,intent)+25.f)continue;
        if(SlaiMeshPath(l,reg,floor,l.meshRoute)){found=true;break;}
    }
    if(!found){
        if(!SlaiMeshLink(l,reg,intent,away,l.meshRoute,l.meshLanding,l.meshJump))return false;
        found=true;
    }
    l.meshMode=mode;l.meshGeneration=NGenNav();l.meshSerial=NNavLinkSerial();l.meshThreat=intent;l.meshUntil=now+8.f;l.meshCheck=0;
    l.lastProgress=now;l.progressPos=l.pos;l.nextProgressCheck=now+0.5f;
    l.freeUntil=0;ResetSlaiPatrolPlan(l);ResetSlaiNav(l);l.verticalApproach=false;
    if(FPrint())std::printf("Slai: native mesh %s, %zu waypoints\n",l.meshJump?"jump link":"walk",l.meshRoute.size());
    return StepSlaiMesh(l,intent,interested,away,false,false,speed,now,dt);
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
        ResetSlaiPatrolPlan(l);l.patrolRetry=now+3.f;
        l.state="defeated";
        if(l.respawnAt==0){l.respawnAt=now+6;RestartClip(l,l.defeat);}
        if(now<l.respawnAt){hold();return;}
        std::printf("Slai: respawn, HP 0 -> %d\n",c_nSlaiMaxHP);
        l.hp=c_nSlaiMaxHP;l.coins=0;l.pickupUntil=0;l.pickupStarted=0;l.pickupSkipUntil=0;l.nextPickupSearch=now;l.respawnAt=0;l.hurtUntil=0;l.nextHit=now+2;l.nextGuardProjectileHit=now+2;
        CancelSlaiBinoculars(l);l.attackUntil=0;l.attackTarget=nullptr;l.nextCombat=0;l.fleeing=false;l.restUntil=0;
        ResetSocial(l,now,false);l.socialPlayer=nullptr;
        l.verticalApproach=false;l.verticalFlight=false;l.nextVertical=now+3;l.jumping=false;l.jumpRecoveryAirborne=false;l.roofSecondPending=false;l.roofFlight=false;l.jumpPath=nullptr;l.nextJump=now+3;l.waterJump=false;l.waterCrossFlight=false;l.nextWaterCrossProbe=now+3;l.waterSince=0;ResetSlaiNav(l);
        l.path=l.spawnPath;l.s=l.spawnS;l.pos=l.spawn;l.heading=l.spawnHeading;
        ResetSlaiMesh(l);l.meshRetry=0;l.floorFitValid=false;l.nextFloorFit=0;
        l.recentPaths.clear();l.failedGoals.clear();l.hasCommandedGoal=false;l.nextReverse=now;l.nextRecovery=0;l.blockedUntil=0;l.freeUntil=0;l.nextLocal=now;l.nextPad=now+3;l.routeSince=now;l.lastProgress=now;l.nextProgressCheck=now+0.5f;l.progressPos=l.spawn;
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
    if(RecoverSlaiWater(psw,l,step,now,dt)){CancelSlaiBinoculars(l);ResetSlaiMesh(l);ResetSlaiPatrolPlan(l);return;}
    if(StepSlaiJump(psw,l,step,now)){CancelSlaiBinoculars(l);ResetSlaiMesh(l);ResetSlaiPatrolPlan(l);if(!l.jumping)return;l.state=l.verticalFlight?l.verticalState:l.roofFlight?(l.roofSecondPending?"roof-jump":"roof-double"):"jump";return;}
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
        ResetSlaiMesh(l);ResetSlaiPatrolPlan(l);l.jumpRecoveryAirborne=false;l.pos=l.safePos;PlaceAt(l.plo,l.pos,l.heading);l.path=nullptr;l.nextSearch=now;l.lastProgress=now;hold();return;
    }
    if(step->pxpFloor)l.jumpRecoveryAirborne=false;
    if(l.jumpRecoveryAirborne && !step->pxpFloor && now>=l.hurtUntil){
        ResetSlaiPatrolPlan(l);l.verticalApproach=false;l.state="airborne-recover";
        step->nMotion=-1;step->vecVel=glm::vec3(0);return;
    }
    if(now<l.hurtUntil){CancelSlaiBinoculars(l);ResetSlaiMesh(l);ResetSlaiPatrolPlan(l);l.verticalApproach=false;l.state="hurt";l.lastProgress=now;l.progressPos=l.pos;l.nextProgressCheck=now+0.5f;hold();return;}
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
    // Trigger only while the actual R2 gesture is active and Slai is inside
    // Carmelita's original discovery cone. Slai's own facing is irrelevant.
    bool threat=false;
    if(carm&&!FPcGuardMoves()&&seen&&d<=c_sSlaiObserve)
        if(STEP* st=PstepFromLo(player))
            threat=FSlaiCarmGestureThreat(true,OidPcPuppetAnim(),
                RadStepHeading(st),eye.x-pp.x,eye.y-pp.y);
    if(!carm||d>=c_sSlaiSafe){l.fleeing=false;ResetSlaiNav(l);}
    else if(threat&&!l.fleeing){
        l.fleeing=true;l.nextDirection=now;l.freeUntil=0;l.nextLocal=now;l.verticalApproach=false;
        ResetSlaiMesh(l);ResetSlaiPatrolPlan(l);ResetSlaiNav(l);
        l.lastProgress=now;l.progressPos=l.pos;l.nextProgressCheck=now+0.5f;
        std::printf("Slai: in Carmelita discovery cone during R2, fleeing\n");
    }
    if(l.fleeing){
        CancelSlaiBinoculars(l);
        if(l.following||l.visiting||l.followHolding)ResetSocial(l,now,true);
        l.attackUntil=0;l.attackTarget=nullptr;l.restUntil=0;
    }
    // Combat persists through attack cooldowns; only an actual approach can translate the actor.
    if(!l.fleeing)
    {
        if(!LiveGuard(l.attackTarget)){if(l.attackTarget)l.nextCombat=now+RandomRange(4,10);l.attackTarget=nullptr;l.attackUntil=0;}
        if(!l.attackTarget&&now>=l.nextCombat&&!l.attacks.empty()){l.attackTarget=NearbyGuard(l.pos);if(l.attackTarget)l.lastGuardSeen=now;}
        if(StepSlaiPickup(l,now,dt,l.attackTarget!=nullptr)){CancelSlaiBinoculars(l);ResetSlaiPatrolPlan(l);l.verticalApproach=false;return;}
        if(l.attackTarget)
        {
            l.verticalApproach=false;
            const glm::vec3 target=PosAloCenter(static_cast<ALO*>(l.attackTarget));const float dist=Distance(eye,target);
            const bool clear=SlaiClearLine(eye,target);if(clear)l.lastGuardSeen=now;
            if(dist>2500||now-l.lastGuardSeen>5.0f){l.attackTarget=nullptr;l.attackUntil=0;l.nextCombat=now+RandomRange(4,10);}
            else
            {
                if(l.visiting||l.following||l.followHolding)ResetSocial(l,now,true);
                CancelSlaiBinoculars(l);ResetSlaiMesh(l);ResetSlaiPatrolPlan(l);l.state="combat";l.restUntil=0;l.lastProgress=now;FaceSlai(l,target,dt);
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
        if(!l.visiting&&!l.following&&now>=l.nextVisit)
        {
            l.nextVisit=now+RandomRange(60,120);
            // Nearby encounters use curiosity, not a second independent roll.
            if(d>c_sSlaiDetect&&d<=c_sSlaiVisitRange&&now>=l.socialCooldown&&
                now>=l.nextCuriosity&&RandomRange(0,1)<c_gSlaiVisitChance)
            {
                l.visiting=true;l.followHolding=false;l.holdUntil=0;l.followDeadline=0;
                l.visitDeadline=now+30;l.visitEnd=0;l.freeUntil=0;l.nextLocal=now;
                l.lastProgress=now;l.progressPos=l.pos;
                std::printf("Slai: spontaneous visit started\n");
            }
        }
        if(seen)l.lastPlayerSeen=now;
        const bool tracking=seen||(l.following&&d<=c_sSlaiDetect&&now-l.lastPlayerSeen<=2.0f);
        const bool wants=l.visiting||StepSlaiCuriosity(l,now,carm&&seen,carm&&tracking);
        if(wants&&!l.following)
        {
            l.following=true;l.followHolding=false;l.holdUntil=0;l.freeUntil=0;l.nextLocal=now;
            l.lastProgress=now;l.progressPos=l.pos;
        }
        else if(!wants&&l.following)
        {
            l.following=false;l.followHolding=false;l.holdUntil=0;l.freeUntil=0;l.nextLocal=now;
            l.verticalApproach=false;l.lastProgress=now;l.progressPos=l.pos;l.nextProgressCheck=now+0.5f;
            l.nextRest=now+SlaiExploreTime();
        }
    }
    const bool follow=validPlayer&&!l.fleeing&&l.following;
    const bool interested=follow||l.fleeing;
    if(interested){CancelSlaiBinoculars(l);ResetSlaiPatrolPlan(l);}
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
            if(!l.followHolding&&d<=stop&&std::abs(pp.z-l.pos.z-65.f)<=125.f&&(!l.visiting||seen))
            {
                l.followHolding=true;l.holdUntil=now+2;l.freeUntil=0;l.nextLocal=now;
                if(l.visiting&&l.visitEnd==0)l.visitEnd=now+RandomRange(10,20);
            }
            if(l.followHolding&&now>=l.holdUntil&&(d>=resume||std::abs(pp.z-l.pos.z-65.f)>125.f))
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
        if(now>=l.nextRest&&now>=l.restUntil){
            CancelSlaiBinoculars(l);
            l.restUntil=now+SlaiRestTime();l.nextRest=l.restUntil+SlaiExploreTime();
            l.restClip=l.idles.empty()?l.idle:l.idles[std::uniform_int_distribution<size_t>(0,l.idles.size()-1)(s_rng)];
            // A single roll per existing rest; never extend a rest or interrupt movement.
            if(step->pxpFloor && l.binocularIn && l.binocularOut &&
                l.restUntil-now>=ClipTime(l.binocularIn,1.f)+ClipTime(l.binocularOut,0.5f)+0.25f && RandomRange(0,1)<0.15f){
                l.binocularPhase=1;l.binocularUntil=now+ClipTime(l.binocularIn,1.f);
                RestartClip(l,l.binocularIn);
            }
        }
        if(now<l.restUntil){ResetSlaiMesh(l);ResetSlaiPatrolPlan(l);l.verticalApproach=false;l.state="rest";hold();
            if(!step->pxpFloor)CancelSlaiBinoculars(l);
            if(!StepSlaiBinoculars(l,now))PlayClip(l,l.restClip);
            l.lastProgress=now;return;}
        CancelSlaiBinoculars(l);
    }
    if(!move){ResetSlaiMesh(l);l.verticalApproach=false;l.state=l.visiting?"visit-rest":"observe";hold();l.actualSpeed=0;PlaySlai(l,false);l.lastProgress=now;l.progressPos=l.pos;return;}
    if(now>=l.nextProgressCheck)
    {
        const glm::vec2 moved(l.pos-l.progressPos);
        if(l.hasCommandedGoal){
            const glm::vec2 toward(l.commandedGoal-l.progressPos);
            const float len=glm::length(toward);
            // Test useful progress along the commanded direction, not sideways sliding.
            const float gain=len-glm::length(glm::vec2(l.commandedGoal-l.pos));
            if((len<30.f&&gain>3.f)||gain>std::min(20.f,len*0.35f))l.lastProgress=now;
        }else if(glm::length(moved)>25.f)l.lastProgress=now;
        l.progressPos=l.pos;l.nextProgressCheck=now+0.5f;
    }
    const bool stuck=now-l.lastProgress>0.85f, old=now-l.routeSince>30.0f;
    l.state=l.fleeing?"flee":l.visiting?"visit":follow?"follow":"explore";
    // Use the current destination rather than an occasional random roof jump.
    glm::vec3 verticalGoal=interested?pp:l.pos+glm::vec3(std::cos(l.heading)*500.f,std::sin(l.heading)*500.f,0);
    if(!interested){
        glm::vec3 routeGoal(0);
        if(c&&SamplePath(c,l.s+l.dir*300.f,routeGoal)&&Distance(routeGoal,l.pos)>65)verticalGoal=routeGoal;
        else if(!c){float ns=0;LO* nearby=FindPatrol(psw,l.pos,ns,nullptr,1200.f);
            if(nearby&&SamplePath(PcrvFromLo(nearby),ns,routeGoal))verticalGoal=routeGoal;}
    }
    if(l.verticalApproach&&!interested&&!l.verticalAway)verticalGoal=l.verticalIntent;
    // Prefer the authored navmesh before the synthetic graph or speculative climbing.
    const bool requestMesh=stuck||(!c&&!interested)||
        (follow&&!l.verticalApproach&&now>=l.meshRetry&&Distance(l.pos,pp)<1200.f&&!GroundConnection(l.pos,verticalGoal));
    if(!l.verticalApproach&&StepSlaiMesh(l,interested?pp:verticalGoal,interested,l.fleeing,stuck,requestMesh,speed,now,dt))return;
    // Keep moving on valid patrol/local routes; use the graph only after real lack of progress.
    if(!interested&&!l.verticalApproach&&l.patrolNodes.empty()&&!l.patrolTarget&&TrySlaiPatrolWater(l,c,now))return;
    if(!interested&&!l.verticalApproach&&StepSlaiPatrolPlan(psw,l,stuck,stuck,now,dt))return;
    const bool recover=stuck&&now>=l.nextRecovery&&l.navNodes.empty()&&l.navRoute.empty()&&!l.verticalApproach;
    if(recover){
        if(l.hasCommandedGoal)RememberSlaiFailure(l,l.commandedGoal,false,now);
        l.hasCommandedGoal=false;
        l.blockedPos=l.pos;l.blockedUntil=now+18;l.freeUntil=0;l.nextLocal=0;l.nextRecovery=now+1.0f;
    }
    if(l.fleeing){
        // Escape is a reactive ground-first task, not an exploration/roof search.
        const bool validFree=now<l.freeUntil&&Distance(l.pos,l.freeGoal)>35.f&&
            GroundConnection(l.pos,l.freeGoal)&&!SlaiFailedGoalNear(l,l.freeGoal,false,now);
        auto runTo=[&](const glm::vec3& q){
            l.verticalApproach=false;SlaiDrive(l,q,speed,dt);
            PlayClip(l,l.run?l.run:l.walk);l.state="flee-ground";
        };
        if(!l.navRoute.empty()&&StepSlaiDetour(l,pp,stuck,now,dt))return;
        if(validFree&&!stuck){ResetSlaiNav(l);runTo(l.freeGoal);return;}
        glm::vec3 candidate(0);
        const bool canProbe=now>=l.nextLocal||(!validFree&&l.freeUntil>0.f);
        bool found=false;
        if(canProbe){
            l.nextLocal=now+0.18f;l.freeUntil=0;
            found=LocalGoal(l,pp,true,false,candidate);
            if(found&&Distance(candidate,pp)>Distance(l.pos,pp)+25.f){
                ResetSlaiNav(l);l.freeGoal=candidate;l.freeUntil=now+0.8f;
                if(stuck){l.lastProgress=now;l.progressPos=l.pos;l.nextProgressCheck=now+0.5f;}
                runTo(candidate);return;
            }
        }
        if(StepSlaiDetour(l,pp,stuck,now,dt))return;
        // A lateral or backwards ground move is preferable to a speculative roof jump.
        if(found){l.freeGoal=candidate;l.freeUntil=now+0.6f;
            if(stuck){l.lastProgress=now;l.progressPos=l.pos;l.nextProgressCheck=now+0.5f;}
            runTo(candidate);return;}
        if(validFree){runTo(l.freeGoal);return;}
        if((stuck||l.verticalApproach)&&StepSlaiVertical(l,pp,true,now,dt))return;
        hold();PlaySlai(l,false);l.state="flee-blocked";return;
    }
    if(l.verticalApproach||((stuck||(follow&&std::abs(pp.z-l.pos.z-65.f)>125.f))&&l.navNodes.empty()&&l.navRoute.empty()))
        if(StepSlaiVertical(l,verticalGoal,l.fleeing,now,dt))return;
    if(recover){
        if(TrySlaiPad(psw,l,interested?&pp:nullptr,l.fleeing,dt))return;
        if(StepSlaiVertical(l,verticalGoal,l.fleeing,now,dt))return;
    }
    // Follow/flee in the world, not merely along a path. Keep waypoints long enough to make progress.
    const bool freeMove=interested||!c||stuck||now<l.freeUntil;
    if(freeMove)
    {
        // Keep a valid waypoint until it is reached or its short lease expires.
        if((interested||!c||stuck)&&(now>=l.nextLocal)&&
            (now>=l.freeUntil||Distance(l.pos,l.freeGoal)<40||!GroundConnection(l.pos,l.freeGoal)||SlaiFailedGoalNear(l,l.freeGoal,false,now)))
        {
            l.nextLocal=now+0.6f;
            if(LocalGoal(l,pp,l.fleeing,!interested,l.freeGoal))l.freeUntil=now+1.5f;
            else l.freeUntil=0;
        }
        if(now<l.freeUntil&&GroundConnection(l.pos,l.freeGoal)&&!SlaiFailedGoalNear(l,l.freeGoal,false,now))
        {
            SlaiDrive(l,l.freeGoal,speed,dt);
            const bool run=l.fleeing&&l.run;
            const float actual=glm::length(glm::vec2(VelAloPos(a)));
            l.actualSpeed=glm::mix(l.actualSpeed,actual,std::min(dt*8,1.0f));
            PlayClip(l,run?l.run:l.walk,true,std::clamp(l.actualSpeed/(run?c_sSlaiRun:c_sSlaiWalk),0.25f,1.15f));
            return;
        }
        if(TrySlaiPad(psw,l,interested?&pp:nullptr,l.fleeing,dt))return;
        if(StepSlaiVertical(l,verticalGoal,l.fleeing,now,dt))return;
    }
    if((old||!c)&&!interested&&TrySlaiPad(psw,l,nullptr,false,dt))return;
    if(!c){
        if(now>=l.nextSwitch){
            l.nextSwitch=now+0.5f;
            if(SwitchPath(psw,l,l.fleeing?&pp:nullptr,true)){c=PcrvFromLo(l.path);l.nextSwitch=now+3.f;}
        }
        if(!c){hold();PlaySlai(l,false);return;}
    }
    const float look=std::clamp(speed*0.20f,55.0f,115.0f);
    glm::vec3 goal(0);
    auto findGoal=[&]{
        for(float distance:{look,look*0.5f,25.0f})
            if(SlaiPatrolSample(c,l.s+l.dir*distance,goal)&&glm::length(glm::vec2(goal-l.pos))>12&&GroundConnection(l.pos,goal)&&(now>=l.blockedUntil||Distance(goal,l.blockedPos)>100)&&!SlaiFailedGoalNear(l,goal,false,now))return true;
        return false;
    };
    bool ok=findGoal();
    bool switched=false;
    if(now>=l.nextSwitch&&(!ok||stuck||old||l.fleeing)){
        l.nextSwitch=now+0.5f;
        if(SwitchPath(psw,l,l.fleeing?&pp:nullptr,old||stuck||!ok)){
            c=PcrvFromLo(l.path);ok=findGoal();switched=ok;l.nextSwitch=now+3.f;
        }
    }
    if(!ok||(stuck&&!switched)){
        if(StepSlaiVertical(l,verticalGoal,l.fleeing,now,dt))return;
        if(now>=l.nextReverse){l.dir=-l.dir;l.nextReverse=now+1.5f;l.nextDirection=now+2.0f;ok=findGoal();}
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
    l.waterClip=clip(225);l.binocularIn=clip(235);l.binocularOut=clip(236);
    // Inventory only: special traversal requires attachment/physics controllers.
    if(std::getenv("SLY2N_SLAICLIPS")){
        for(int oid:{172,174,189,171,257,260,149,154,147,158,146,4566,152,153,161,160,225,222,235,236})
            std::printf("Slai: ASEG %d %s\n",oid,clip(oid)?"found":"missing");
    }
    // Native resolver checks the three K2 event markers. Never play an unverified guard clip on Sly.
    l.nativeJumpTiming=nullptr;
    for(ASEG* candidate:{l.jumpClip,l.doubleJumpClip})if(candidate){
        const KIND2* k=Pk2ResolveAlo(palo,candidate);
        const bool valid=k&&std::isfinite(k->dt14)&&k->t22<k->t23&&k->t23<k->t24&&k->dt14>=0.25f&&k->dt14<=2.f;
        std::printf("Slai: ASEG %d native K2 markers %s\n",candidate->oid,valid?"compatible":"missing/invalid");
        if(valid&&candidate==l.jumpClip)l.nativeJumpTiming=k;
    }
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

// Secondary activation viewpoint. Never changes the current player or camera.
bool FModSlaiActivationCenter(const SW* psw, glm::vec3* out)
{
    if (!psw || psw != g_psw || psw->serial != s_serial || !out) return false;
    for (const LIVE& l : s_alive)
    {
        if (!l.fSlai || !l.plo || !l.hasSpawn || l.hp <= 0 || !l.plo->fInWorld || l.plo->fRemoved) continue;
        const glm::vec3 p = SlaiFeet(l.plo);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        *out = p;
        return true;
    }
    return false;
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
        const bool projectile=FCidDerivedFrom(hit->ploSender->Cid(),0x4F);
        LO* launcher=projectile?PloProjectileLauncherActor(hit->ploSender):nullptr;
        const bool guardShot=!electric&&launcher&&FModSlaiOrdinaryGuard(launcher);
        const float now=g_clock.t;
        const char* rejected=nullptr;
        if (!l.hasSpawn || l.hp<=0) rejected="not active";
        else if (FGamePausedForAi()) rejected="paused";
        else if (g_clock.t<l.nextHit) rejected="immunity";
        else if(hit->nState>=0 || hit->grfFilter<=1 || hit->grfFilter==0x100000 || hit->grfFilter==0x80000) rejected="not damage";
        else if(guardShot && now<l.nextGuardProjectileHit) rejected="guard shot recovery";
        // Raw contact times (including rejected contacts), not a count of unique shots fired.
        if(guardShot && std::getenv("SLY2N_SLAIGUARD"))
            std::printf("Slai: guard-shot contact t=%.3f guard=%d projectile=%d result=%s next=%.3f\n",now,launcher->oid,hit->ploSender->oid,rejected?rejected:"accepted",l.nextGuardProjectileHit);
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
        CancelSlaiBinoculars(l);l.nextHit=g_clock.t+1.0f; l.attackUntil=0; l.restUntil=0;
        if(l.visiting||l.following||l.followHolding)ResetSocial(l,g_clock.t,true);
        if(STEP* step=PstepFromLo(l.plo)) step->vecVel=glm::vec3(0);
        LO* attacker=hit->ploSender;
        if(!LiveGuard(attacker)&&projectile)attacker=launcher;
        if(LiveGuard(attacker)){l.attackTarget=attacker;l.lastGuardSeen=g_clock.t;l.nextCombat=0;}
        if (l.hp <= 0) { l.respawnAt=g_clock.t+6.0f; RestartClip(l,l.defeat); }
        else { ASEG* reaction=electric && l.shock?l.shock:l.hurt;
            float stun=ClipTime(reaction,0.6f);
            if(guardShot){
                STEP* step=PstepFromLo(l.plo);
                stun=SlaiGuardShotStun(stun,l.jumping||!step||!step->pxpFloor);
            }
            l.hurtUntil=now+stun;
            if(!electric)l.nextHit=std::max(l.nextHit,l.hurtUntil+0.75f); // preserve existing common recovery
            if(guardShot)l.nextGuardProjectileHit=SlaiGuardShotDeadline(now,l.hurtUntil);
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
