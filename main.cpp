#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <d3d12.h>
#include <dxgi1_6.h>
#include <d3dcompiler.h>
#if defined(__has_include)
#if __has_include(<dxcapi.h>)
#include <dxcapi.h>
#define HAVE_DXC 1
#endif
#endif
#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <climits>
#include <algorithm>
#include <functional>
#include <random>
#include <future>
#include <mutex>
#include <atomic>
#include <thread>
#include "resource.h"


static_assert(sizeof(void*) == 8, "High Visual Edition は 64bit 版専用です。x64 でビルドしてください (Visual Edition までは 32bit でも動きます)");

#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3dcompiler.lib")


static const float PI = 3.14159265358979f;
struct V3 { float x, y, z; };
static V3 operator+(V3 a, V3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
static V3 operator-(V3 a, V3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
static V3 operator*(V3 a, float s) { return { a.x * s, a.y * s, a.z * s }; }
static float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static V3 cross(V3 a, V3 b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
static V3 norm(V3 a) { float l = sqrtf(dot(a, a)); return l > 0 ? a * (1.0f / l) : a; }

struct M4 { float m[4][4]; };
static M4 ident() { M4 r{}; for (int i = 0; i < 4; i++) r.m[i][i] = 1; return r; }
static M4 mul(const M4& a, const M4& b) {
	M4 r{};
	for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) for (int k = 0; k < 4; k++) r.m[i][j] += a.m[i][k] * b.m[k][j];
	return r;
}
static M4 lookAt(V3 e, V3 at, V3 up) {
	V3 z = norm(at - e), x = norm(cross(up, z)), y = cross(z, x);
	M4 m = ident();
	m.m[0][0] = x.x; m.m[1][0] = x.y; m.m[2][0] = x.z;
	m.m[0][1] = y.x; m.m[1][1] = y.y; m.m[2][1] = y.z;
	m.m[0][2] = z.x; m.m[1][2] = z.y; m.m[2][2] = z.z;
	m.m[3][0] = -dot(x, e); m.m[3][1] = -dot(y, e); m.m[3][2] = -dot(z, e);
	return m;
}
static M4 perspective(float fov, float aspect, float zn, float zf) {
	float ys = 1.0f / tanf(fov * 0.5f), xs = ys / aspect;
	M4 m{};
	m.m[0][0] = xs; m.m[1][1] = ys;
	m.m[2][2] = zf / (zf - zn); m.m[2][3] = 1;
	m.m[3][2] = -zn * zf / (zf - zn);
	return m;
}
static M4 basisMat(V3 r, V3 u, V3 f, V3 p) {
	M4 m = ident();
	m.m[0][0] = r.x; m.m[0][1] = r.y; m.m[0][2] = r.z;
	m.m[1][0] = u.x; m.m[1][1] = u.y; m.m[1][2] = u.z;
	m.m[2][0] = f.x; m.m[2][1] = f.y; m.m[2][2] = f.z;
	m.m[3][0] = p.x; m.m[3][1] = p.y; m.m[3][2] = p.z;
	return m;
}


struct Pose { V3 p, f; };

enum SegmentType { SEG_STRAIGHT, SEG_CURVE_LEFT, SEG_CURVE_RIGHT };
struct TrackSegment {
	SegmentType type;
	float len;
	float radius;
	float startS;
	float endS;
	V3 startPos;
	V3 startDir;
};

struct StationData {
	std::wstring name;
	float stopS = 0.0f;
	float platformS0 = 0.0f, platformS1 = 0.0f;
	float stopSUp = 0.0f;
	bool big = false;
};

static std::vector<TrackSegment> g_segments;
static std::vector<StationData> g_stations;
static float g_totalTrackLen = 0.0f;

static const int   NUM_CARS = 4;
static const float CAR_LEN = 20.0f, CAR_GAP = 0.5f, BOGIE_HALF = 7.0f;
static const float TRAIN_LEN = NUM_CARS * CAR_LEN + (NUM_CARS - 1) * CAR_GAP;
static const float CAR_Y = 0.58f;
static const double DEPART_TOL = 3.0;

static V3 rightOf(V3 f) { return { f.z, 0, -f.x }; }

static float trackHeightRaw(float s) {
	float ratio = s / g_totalTrackLen;
	return sinf(ratio * 2.0f * PI * 3.0f) * 20.0f + cosf(ratio * 2.0f * PI * 5.0f) * 8.0f;
}
static const float STN_FLAT_R0 = 55.0f, STN_FLAT_R1 = 110.0f, STN_FLAT_K = 0.7f;
static float trackHeight(float s) {
	float raw = trackHeightRaw(s), h = raw;
	for (const auto& st : g_stations) {
		const float R0 = st.big ? 480.0f : STN_FLAT_R0, R1 = st.big ? 660.0f : STN_FLAT_R1, K = st.big ? 0.92f : STN_FLAT_K;
		float c = (st.platformS0 + st.platformS1) * 0.5f;
		float d = fmodf(fabsf(s - c), g_totalTrackLen); d = std::min(d, g_totalTrackLen - d);
		if (d >= R1) continue;
		float t = std::min(1.0f, std::max(0.0f, (R1 - d) / (R1 - R0)));
		float w = t * t * (3.0f - 2.0f * t);
		h -= (1.0f - K) * w * (raw - trackHeightRaw(c));
	}
	return h;
}

static void generateProceduralWorld() {
	g_segments.clear();
	g_stations.clear();

	float curS = 0.0f;
	int numSegments = 40;
	float radius = 600.0f;
	float totalAngleNeeded = 2.0f * PI;
	float angleStep = totalAngleNeeded / numSegments;
	float segLen = radius * angleStep;

	V3 curPos = { 0, 0, 0 };
	float currentAngle = 0.0f;

	std::random_device rd;
	std::mt19937 rng(rd());

	const wchar_t* stPrefix[] = { L"新", L"東", L"西", L"南", L"北", L"中央", L"緑ヶ丘", L"海岸", L"学園前", L"夢見" };
	const wchar_t* stSuffix[] = { L"町", L"市", L"公園", L"橋", L"台", L"原", L"関", L"川" };

	for (int i = 0; i < numSegments; i++) {
		TrackSegment seg;
		seg.startS = curS;
		seg.len = segLen;
		seg.radius = radius;
		seg.startPos = curPos;

		V3 curDir = { cosf(currentAngle), 0, sinf(currentAngle) };
		seg.startDir = curDir;

		seg.type = SEG_CURVE_LEFT;
		float dTheta = angleStep;
		currentAngle += dTheta;



		V3 r = rightOf(curDir);
		V3 center = curPos - r * seg.radius;
		float angle = seg.len / seg.radius;

		float cosA = cosf(angle), sinA = sinf(angle);
		V3 newDir = norm({ curDir.x * cosA - curDir.z * sinA, 0, curDir.x * sinA + curDir.z * cosA });
		V3 newR = rightOf(newDir);
		curPos = center + newR * seg.radius;

		if (i == numSegments - 1) {
			curPos = { 0, 0, 0 };
		}

		seg.endS = curS + seg.len;
		g_segments.push_back(seg);
		curS += seg.len;

		if (i == 0 || i == 8 || i == 13 || i == 18 || i == 23 || i == 28 || i == 32) {
			StationData st;
			wchar_t sname[64];
			swprintf_s(sname, L"%s%s駅", stPrefix[rng() % 10], stSuffix[rng() % 8]);
			st.name = sname;
			st.platformS0 = seg.startS + 10.0f;
			st.platformS1 = st.platformS0 + 120.0f;
			st.stopS = st.platformS1 - 15.0f;
			st.stopSUp = st.platformS0 + 15.0f;
			st.big = (i == 0);
			g_stations.push_back(st);
		}
	}
	g_totalTrackLen = curS;
}

static Pose trackAt(float s) {
	s = fmodf(s, g_totalTrackLen);
	if (s < 0) s += g_totalTrackLen;

	const TrackSegment* seg = &g_segments[0];
	for (const auto& sg : g_segments) {
		if (s >= sg.startS && s < sg.endS) {
			seg = &sg;
			break;
		}
	}

	float localS = s - seg->startS;
	Pose o{};
	if (seg->type == SEG_STRAIGHT) {
		o.p = seg->startPos + seg->startDir * localS;
		o.f = seg->startDir;
	}
	else {
		V3 r = rightOf(seg->startDir);
		V3 center = seg->startPos - r * seg->radius;
		float angle = localS / seg->radius;

		float cosA = cosf(angle), sinA = sinf(angle);
		o.f = norm({ seg->startDir.x * cosA - seg->startDir.z * sinA, 0, seg->startDir.x * sinA + seg->startDir.z * cosA });
		V3 curR = rightOf(o.f);
		o.p = center + curR * seg->radius;
	}
	o.p.y = trackHeight(s);
	return o;
}


struct Vtx { V3 p, n, c; float e; };
using Verts = std::vector<Vtx>;
static float g_curE = 0.0f;
static std::vector<V3> g_lampPos;

static void addBox(Verts& v, V3 c, V3 r, V3 u, V3 f, V3 h, V3 col) {
	V3 ax[3] = { r, u, f }; float hs[3] = { h.x, h.y, h.z };
	for (int k = 0; k < 3; k++) for (int sg = -1; sg <= 1; sg += 2) {
		int a = (k + 1) % 3, b = (k + 2) % 3;
		V3 n = ax[k] * (float)sg, ca = c + n * hs[k];
		V3 A = ax[a] * hs[a], B = ax[b] * hs[b];
		V3 q[4] = { ca - A - B, ca + A - B, ca + A + B, ca - A + B };
		int idx[6] = { 0, 1, 2, 0, 2, 3 };
		for (int i : idx) v.push_back({ q[i], n, col, g_curE });
	}
}
static void addAABB(Verts& v, V3 c, V3 h, V3 col) { addBox(v, c, { 1,0,0 }, { 0,1,0 }, { 0,0,1 }, h, col); }


struct TFrame { V3 p, r, u, f; };
static TFrame trackFrame(float s) {
	Pose P = trackAt(s), A = trackAt(s + 0.5f), B = trackAt(s - 0.5f);
	V3 f = norm(A.p - B.p), r0 = rightOf(P.f);
	if (dot(f, P.f) < 0.0f) f = f * -1.0f;
	V3 u = norm(cross(f, r0));
	V3 r = norm(cross(u, f));
	return { P.p, r, u, f };
}
struct PPt { float x, y; };
static void pushV(Verts& v, V3 p, V3 n, V3 c) { v.push_back({ p, n, c, g_curE }); }

static void sweepProfileF(Verts& v, const std::vector<PPt>& pf, const std::vector<V3>& ec, float s0, float s1, float ds, const std::function<float(float)>& latF, float lift, bool caps, V3 capCol) {
	const int n = (int)pf.size();
	const int rings = std::max(1, (int)ceilf((s1 - s0) / ds));
	const float step = (s1 - s0) / rings;
	auto at = [&](const TFrame& F, const PPt& q, float lat) { return F.p + F.r * (q.x + lat) + F.u * (q.y + lift); };
	TFrame F0 = trackFrame(s0); float l0 = latF(s0);
	if (caps) {
		for (int e = 0; e < 2; e++) {
			const float se = (e == 0) ? s0 : s1;
			TFrame F = (e == 0) ? F0 : trackFrame(s1); const float le = (e == 0) ? l0 : latF(s1);
			V3 nn = (e == 0) ? F.f * -1.0f : F.f;
			for (int i = 1; i + 1 < n; i++) {
				pushV(v, at(F, pf[0], le), nn, capCol); pushV(v, at(F, pf[i], le), nn, capCol); pushV(v, at(F, pf[i + 1], le), nn, capCol);
			}
			(void)se;
		}
	}
	for (int i = 0; i < rings; i++) {
		const float sN = s0 + (i + 1) * step;
		TFrame F1 = trackFrame(sN); const float l1 = latF(sN);
		for (int e = 0; e < n; e++) {
			const PPt a = pf[e], b = pf[(e + 1) % n];
			float ex = b.x - a.x, ey = b.y - a.y, el = sqrtf(ex * ex + ey * ey);
			if (el < 1e-6f) continue;
			float nx = ey / el, ny = -ex / el;
			V3 n0 = norm(F0.r * nx + F0.u * ny), n1 = norm(F1.r * nx + F1.u * ny);
			V3 p00 = at(F0, a, l0), p01 = at(F0, b, l0), p10 = at(F1, a, l1), p11 = at(F1, b, l1);
			const V3 c = ec[e];
			pushV(v, p00, n0, c); pushV(v, p01, n0, c); pushV(v, p11, n1, c);
			pushV(v, p00, n0, c); pushV(v, p11, n1, c); pushV(v, p10, n1, c);
		}
		F0 = F1; l0 = l1;
	}
}
static void sweepProfile(Verts& v, const std::vector<PPt>& pf, const std::vector<V3>& ec, float s0, float s1, float ds, float lat, float lift, bool caps, V3 capCol) {
	sweepProfileF(v, pf, ec, s0, s1, ds, [lat](float) { return lat; }, lift, caps, capCol);
}



static const float DOOR_Z[3] = { -6.0f, 0.0f, 6.0f };
static const float DOOR_HW = 0.65f;
static const float LEAF_HW = 0.32f;
static const float LEAF_OFS = 0.325f;
static const float LEAF_SLIDE = 0.64f;
static const float LEAF_PLUG = 0.055f;
static const float Y_SKIN0 = 0.90f, Y_WHITE1 = 1.75f, Y_FLOOR = 1.10f;
static const float Y_WIN0 = 2.00f, Y_WIN1 = 3.10f, Y_SKIN1 = 3.35f;
static const float X_OUT = 1.40f, X_IN = 1.35f;





static const double TOD_RATE = 24.0 / 2400.0;
static const double RAIL_GPR = TOD_RATE * 3600.0;
static const int   NTRK = 4;
static const float TRK_BASE[NTRK] = { -6.0f, -2.0f, 2.0f, 6.0f };
static const int   TRK_DIR[NTRK] = { 1, -1, 1, -1 };
static const float PF_EDGE = 1.7f, PF_TOP = 1.67f, ISL_W = 8.0f, SIDE_W = 5.0f;
static const float BIG_SHIFT = ISL_W + 2.0f * PF_EDGE - (TRK_BASE[1] - TRK_BASE[0]);
static const float BIG_TRANS = 80.0f;
static const int   NLINE = 2;
static const int   LINE_TRK[NLINE] = { 0, 3 };
static const int   NYT = 4, YCAP = 3;
static const float YARD_LEN = 90.0f;
static const float SLOT_PITCH = TRAIN_LEN + 6.5f;
static const int   NTYPE = 2;
enum { TT_LOCAL = 0, TT_EXPRESS = 1 };
static const wchar_t* TYPE_NAME[NTYPE] = { L"普通", L"急行" };

static float rsm(float t) { t = std::min(1.0f, std::max(0.0f, t)); return t * t * (3.0f - 2.0f * t); }
static float wrapS(float s) { float L = g_totalTrackLen; s = fmodf(s, L); if (s < 0) s += L; return s; }
static float relS(float s) { float L = g_totalTrackLen; s = fmodf(s, L); if (s < 0) s += L; return s > L * 0.5f ? s - L : s; }

static float bigWeight(float s) {
	const StationData& st = g_stations[0];
	const float c = (st.platformS0 + st.platformS1) * 0.5f, half = (st.platformS1 - st.platformS0) * 0.5f + 5.0f;
	const float d = std::max(0.0f, fabsf(relS(s - c)) - half);
	return 1.0f - rsm(d / BIG_TRANS);
}
static float baseLat(int trk, float s) {
	if (trk == 0) return TRK_BASE[0] - BIG_SHIFT * bigWeight(s);
	if (trk == 3) return TRK_BASE[3] + BIG_SHIFT * bigWeight(s);
	return TRK_BASE[trk];
}

struct YardDef { int line = 0, trk = 0, side = 1; float b[NYT] = {}, T[NYT] = {}; };
static YardDef g_yard[NLINE];
struct PathDef { int base = 0, yard = -1, j = 0; };
static std::vector<PathDef> g_paths;
static int yardPath(int line, int j) { return NTRK + line * NYT + j; }
static float pathLat(int pid, float s) {
	const PathDef& p = g_paths[pid];
	const float bl = baseLat(p.base, s);
	if (p.yard < 0) return bl;
	const YardDef& Y = g_yard[p.yard];
	const float u = (float)Y.side * relS(s - Y.b[p.j]);
	if (u <= 0.0f) return bl;
	return bl + (Y.T[p.j] - bl) * rsm(u / YARD_LEN);
}
static float slotHeadInto(int line, int j, int k) {
	const YardDef& Y = g_yard[line];
	return wrapS(Y.b[j] + Y.side * (YARD_LEN + 6.0f + TRAIN_LEN + k * SLOT_PITCH));
}

static std::vector<float> g_sig[NTRK];
static std::vector<float> g_crossS;
static int g_order[NLINE][8];
static int g_nStops = 0;
static float g_cycleHome = 0.0f;

static float stopHead(int line, int stn) { return LINE_TRK[line] == 0 ? g_stations[stn].stopS : g_stations[stn].stopSUp; }
static bool doorsRightAt(int stn) { return stn == 0; }

static void railLayoutInit() {
	const int ns = (int)g_stations.size();
	g_nStops = ns;
	for (int i = 0; i < ns - 1; i++) { g_order[0][i] = i + 1; g_order[1][i] = ns - 1 - i; }
	g_order[0][ns - 1] = 0; g_order[1][ns - 1] = 0;
	g_paths.assign(NTRK + NLINE * NYT, PathDef());
	for (int t = 0; t < NTRK; t++) g_paths[t].base = t;
	for (int l = 0; l < NLINE; l++) {
		YardDef& Y = g_yard[l]; Y.line = l; Y.trk = LINE_TRK[l]; Y.side = -TRK_DIR[Y.trk];
		const float bs = (l == 0) ? -85.0f : 250.0f;
		for (int j = 0; j < NYT; j++) {
			Y.b[j] = bs + Y.side * 25.0f * (NYT - 1 - j);
			Y.T[j] = (Y.trk == 0 ? -1.0f : 1.0f) * (12.5f + 4.6f * j);
			PathDef& p = g_paths[yardPath(l, j)]; p.base = Y.trk; p.yard = l; p.j = j;
		}
	}

	for (int t = 0; t < NTRK; t++) {
		g_sig[t].clear();
		std::vector<float> v;
		for (int k = 0; k < ns; k++) {
			const StationData& a = g_stations[k]; const StationData& b = g_stations[(k + 1) % ns];
			float bs0 = b.platformS0 + ((k + 1 == ns) ? g_totalTrackLen : 0.0f);
			const float mid = (a.platformS1 + bs0) * 0.5f;
			if (TRK_DIR[t] > 0) { v.push_back(a.platformS0 - 30.0f); v.push_back(a.platformS1 + 8.0f); v.push_back(mid); }
			else { v.push_back(a.platformS1 + 30.0f); v.push_back(a.platformS0 - 8.0f); v.push_back(mid); }
		}
		for (float& x : v) x = wrapS(x);
		std::sort(v.begin(), v.end());
		if (TRK_DIR[t] < 0) std::reverse(v.begin(), v.end());
		g_sig[t] = v;
	}

	g_crossS.clear();
	for (int k = 0; k < ns; k++) {
		const StationData& a = g_stations[k]; const StationData& b = g_stations[(k + 1) % ns];
		float b0 = b.platformS0 + ((k + 1 == ns) ? g_totalTrackLen : 0.0f);
		float c = (a.platformS1 + b0) * 0.5f;
		if (k == 0) c = b0 - 42.0f;
		if (k == ns - 1) c = a.platformS1 + 54.0f;
		g_crossS.push_back(wrapS(c));
	}
}


enum { TS_YARD, TS_PULL, TS_ORIGIN, TS_RUN, TS_DWELL, TS_TERM, TS_TOYARD };
struct Train {
	int id = 0, line = 0, type = TT_LOCAL;
	int dir = 1; float s = 0.0f, v = 0.0f; int path = 0;
	int st = TS_YARD; int trip = -1; int pos = 0;
	int yTrk = 0, ySlot = 0;
	int dwStage = 0; float dwTimer = 0.0f;
	float doorL = 0.0f, doorR = 0.0f; bool tgtL = false, tgtR = false;
	float cabPos[4] = { 0, 0, 0, 0 }; bool cabTgt[4] = { false, false, false, false };
	float acc = 0.0f;
	bool onboard = false;
	double depClock = 0.0;
	float tripT = 0.0f;
	float stuckT = 0.0f;
	float arrAt[8] = {}, depAt[8] = {};
};
struct Crossing { float s = 0, arm = 0, timer = 0; int state = 0; bool warn = false; };

struct TimeInfo { float arr[8] = {}, dep[8] = {}; float run = 0; };
struct RailSim {
	std::vector<Train> tr;
	std::vector<Crossing> cx;
	double clock = 0.0;
	double simT = 0.0;
	std::vector<double> trips[NLINE];
	int nextTrip[NLINE] = { 0, 0 };
	int nextPull[NLINE] = { 0, 0 };
	int fleet[NLINE] = { 10, 10 };
	int yardOcc[NLINE][NYT][YCAP] = {};
	int platRes[NLINE] = { -1, -1 };
	unsigned char aspect[NTRK][64] = {};
	bool blockOcc[NTRK][64] = {};
	float extraDwell[NLINE] = { 0, 0 };
	TimeInfo ti[NLINE];
	float pullReal[NLINE] = { 30, 30 };
	int day = 0;
	int checkDay = -1;
	double calArr[NLINE][8] = {}, calDep[NLINE][8] = {}; int calN[NLINE] = { 0, 0 };
};
static RailSim g_rs;

static const float V_MAX = 100.0f / 3.6f, V_SHUNT = 40.0f / 3.6f, V_PULL = 60.0f / 3.6f, A_BRK = 1.10f, A_PAT = 0.95f;
static const float DW_WAIT = 0.5f, DW_OPEN = 2.5f, DW_HOLD = 4.5f, DW_CLOSE = 2.5f, DW_POST = 1.0f;
static const double HEADWAY_G = 30.0 * 60.0, FIRST_DEP_G = 5.0 * 3600.0, LAST_ARR_G = 24.0 * 3600.0;

static float accelAt(float v) {
	const float A0 = 4.0f / 3.6f, V1 = 15.0f, V2 = 24.0f;
	return (v <= V1) ? A0 : (v <= V2) ? A0 * V1 / v : A0 * (V1 / V2) * (V2 / v) * (V2 / v);
}
static int trainBase(const Train& t) { return g_paths[t.path].base; }
static void bodyInterval(const Train& t, float& start, float& len) { len = TRAIN_LEN; start = (t.dir > 0) ? t.s - TRAIN_LEN : t.s; }

static void clipMainZone(int path, float& start, float& len) {
	const PathDef& p = g_paths[path];
	if (p.yard < 0) return;
	const YardDef& Y = g_yard[p.yard];
	const float edge = Y.b[p.j] + Y.side * YARD_LEN * 0.6f;
	const float ds = relS(edge - start);
	if (Y.side > 0) len = std::max(0.0f, std::min(len, ds));
	else if (ds > 0.0f) { start += ds; len = std::max(0.0f, len - ds); }
}
static void mainInterval(const Train& t, float& start, float& len) { bodyInterval(t, start, len); clipMainZone(t.path, start, len); }
static bool ivOverlap(float a0, float al, float b0, float bl, float margin) {
	const float d = relS(b0 - a0);
	return d < al + margin && d + bl > -margin;
}
static bool trainsConflict(const Train& a, const Train& b, float margin) {
	float a0, al, b0, bl;
	if (a.path == b.path) { bodyInterval(a, a0, al); bodyInterval(b, b0, bl); return ivOverlap(a0, al, b0, bl, margin); }
	if (trainBase(a) != trainBase(b)) return false;
	mainInterval(a, a0, al); mainInterval(b, b0, bl);
	if (al <= 0.0f || bl <= 0.0f) return false;
	return ivOverlap(a0, al, b0, bl, margin);
}
static float aheadDist(const Train& t, float target) {
	float d = (float)t.dir * relS(target - t.s);
	return d;
}
static void flipTrain(Train& t) { t.s = wrapS(t.s - (float)t.dir * TRAIN_LEN); t.dir = -t.dir; }

static int blockIndexAt(int trk, float s) {
	const std::vector<float>& sg = g_sig[trk]; const int n = (int)sg.size();
	const float dir = (float)TRK_DIR[trk];
	for (int k = 0; k < n; k++) {
		const float a = sg[k], b = sg[(k + 1) % n];
		const float len = dir * relS(b - a), off = dir * relS(s - a);
		const float L = (len > 0) ? len : len + g_totalTrackLen;
		const float o = (off >= 0) ? off : off + g_totalTrackLen;
		if (o < L) return k;
	}
	return 0;
}

static void blockRange(int trk, int k, float& start, float& len) {
	const std::vector<float>& sg = g_sig[trk]; const int n = (int)sg.size(); const float dir = (float)TRK_DIR[trk];
	const float a = sg[k], b = sg[(k + 1) % n];
	len = dir * relS(b - a); if (len <= 0.0f) len += g_totalTrackLen;
	start = (dir > 0) ? a : a - len;
}
static void computeSignals() {
	RailSim& R = g_rs;
	for (int t = 0; t < NTRK; t++) {
		const int n = (int)g_sig[t].size();
		for (int k = 0; k < n; k++) R.blockOcc[t][k] = false;
		for (const Train& tr : R.tr) {
			if (trainBase(tr) != t) continue;
			float s0, l0; mainInterval(tr, s0, l0);
			if (l0 <= 0.0f) continue;
			for (int k = 0; k < n; k++) {
				float bs, bl; blockRange(t, k, bs, bl);
				if (ivOverlap(bs, bl, s0, l0, 0.0f)) R.blockOcc[t][k] = true;
			}
		}
	}
	for (const Train& o : R.tr) if (o.st == TS_TERM || o.st == TS_TOYARD || o.st == TS_PULL) {
		const int t = LINE_TRK[o.line];
		const float home = (TRK_DIR[t] > 0) ? g_stations[0].platformS0 - 30.0f : g_stations[0].platformS1 + 30.0f;
		R.blockOcc[t][blockIndexAt(t, wrapS(home - (float)TRK_DIR[t] * 1.0f))] = true;
	}
	for (int t = 0; t < NTRK; t++) {
		const int n = (int)g_sig[t].size();
		for (int k = 0; k < n; k++) {
			int a;
			if (R.blockOcc[t][k]) a = 0;
			else if (R.blockOcc[t][(k + 1) % n]) a = 1;
			else if (R.blockOcc[t][(k + 2) % n]) a = 2;
			else a = 3;
			R.aspect[t][k] = (unsigned char)a;
		}
	}
}

static float stopTargetHead(const Train& t) {
	if (t.st == TS_RUN) return stopHead(t.line, g_order[t.line][t.pos]);
	if (t.st == TS_PULL) return stopHead(t.line, 0);
	if (t.st == TS_TOYARD) return slotHeadInto(t.line, t.yTrk, t.ySlot);
	return t.s;
}
static void setDoors(Train& t, bool open) {
	const bool right = doorsRightAt(t.st == TS_ORIGIN || t.st == TS_PULL ? 0 : g_order[t.line][std::min(t.pos, g_nStops - 1)]);
	t.tgtL = open && !right; t.tgtR = open && right;
}
static void stepDoors(Train& t, float dt) {
	const float k = dt / 2.5f;
	t.doorL = std::min(1.0f, std::max(0.0f, t.doorL + (t.tgtL ? k : -k)));
	t.doorR = std::min(1.0f, std::max(0.0f, t.doorR + (t.tgtR ? k : -k)));
}
static int curStop(const Train& t) { return (t.st == TS_ORIGIN) ? 0 : g_order[t.line][std::min(t.pos, g_nStops - 1)]; }


static bool dwellCycle(Train& t, float dt, double clockNow, bool isOrigin) {
	RailSim& R = g_rs;
	t.dwTimer += dt;
	switch (t.dwStage) {
	case 0: if (t.dwTimer >= DW_WAIT) { setDoors(t, true); t.dwStage = 1; t.dwTimer = 0; } break;
	case 1: if (t.dwTimer >= DW_OPEN) { t.dwStage = 2; t.dwTimer = 0; } break;
	case 2: {
		bool close;
		if (isOrigin) close = (clockNow >= t.depClock - (DW_CLOSE + DW_POST + 0.5) * RAIL_GPR) && t.dwTimer >= 4.0f;
		else { float hold = DW_HOLD + R.extraDwell[t.line] / (float)(g_nStops - 1); close = t.dwTimer >= hold; }
		if (t.st == TS_TERM) close = t.dwTimer >= 8.0f && !t.onboard;
		if (close) { setDoors(t, false); t.dwStage = 3; t.dwTimer = 0; }
		break;
	}
	case 3: if (t.doorL <= 0.0f && t.doorR <= 0.0f && t.dwTimer >= DW_CLOSE) { t.dwStage = 4; t.dwTimer = 0; } break;
	case 4: if (t.dwTimer >= DW_POST) { t.dwStage = 5; } break;
	}
	return t.dwStage == 5;
}

static void startRun(Train& t) {
	t.st = TS_RUN; t.pos = 0; t.dwStage = 0; t.dwTimer = 0; t.tripT = 0.0f;
}

static bool tripClaimable(int line) {
	const RailSim& R = g_rs; const int k = R.nextTrip[line];
	return k < (int)R.trips[line].size() && R.day * 86400.0 + R.trips[line][k] - R.clock <= 90.0 * 60.0;
}
static void claimTripOrYard(Train& t) {
	RailSim& R = g_rs;
	int k = R.nextTrip[t.line];

	if (tripClaimable(t.line)) {
		R.nextTrip[t.line]++; t.trip = k; t.depClock = R.day * 86400.0 + R.trips[t.line][k];
		t.st = TS_ORIGIN; t.dwStage = 0; t.dwTimer = 0;
	}
	else { t.trip = -1; t.st = TS_TERM; t.dwStage = 0; t.dwTimer = 0; }
}
static void onArrive(Train& t) {
	RailSim& R = g_rs;
	t.v = 0.0f;
	if (t.st == TS_RUN) {
		t.arrAt[t.pos] = t.tripT;
		t.dwStage = 0; t.dwTimer = 0;
		if (t.pos == g_nStops - 1) {
			if (t.trip >= R.fleet[t.line] / 2 && t.trip >= 0) {
				for (int p = 0; p < g_nStops; p++) { R.calArr[t.line][p] += t.arrAt[p]; R.calDep[t.line][p] += t.depAt[p]; }
				R.calN[t.line]++;
			}
			claimTripOrYard(t);
		}
		else t.st = TS_DWELL;
	}
	else if (t.st == TS_PULL) {
		t.path = LINE_TRK[t.line];
		t.st = TS_ORIGIN; t.dwStage = 0; t.dwTimer = 0;
	}
	else if (t.st == TS_TOYARD) {
		flipTrain(t); t.st = TS_YARD; t.dwStage = 0;
		R.yardOcc[t.line][t.yTrk][t.ySlot] = t.id;
		t.doorL = t.doorR = 0.0f; t.tgtL = t.tgtR = false;
	}
}


static const float STUCK_SEC = 90.0f;
static int g_railRecoverCount = 0;


static bool yardBoundNow(int line) { return !tripClaimable(line); }
static float yardWallS(int line) { const YardDef& Y = g_yard[line]; return wrapS(Y.b[0] + Y.side * YARD_LEN * 0.6f); }
static bool yardWallActive(const Train& t) {
	if (!yardBoundNow(t.line)) return false;
	for (const Train& o : g_rs.tr) {
		if (o.id == t.id || o.line != t.line) continue;
		bool on = false;
		if (o.st == TS_TERM) on = true;
		else if (o.st == TS_TOYARD) { float a, l; mainInterval(o, a, l); on = l > 0.0f; }
		else if (o.st == TS_RUN && o.pos >= g_nStops - 1) on = true;
		if (!on) continue;
		if ((float)t.dir * relS(o.s - t.s) > 0.0f) return true;
	}
	return false;
}


static bool yardMoveClear(const Train& t) {
	const float wall = yardWallS(t.line);
	for (const Train& o : g_rs.tr) {
		if (o.id == t.id || o.line != t.line || (o.st != TS_RUN && o.st != TS_PULL)) continue;
		const float dTerm = aheadDist(o, t.s), dEdge = aheadDist(o, wall);
		const float brake = o.v * o.v / (2.0f * 0.9f) + 8.0f;
		if (dTerm > 0.0f && dTerm < 2500.0f && dEdge < brake) return false;
	}
	return true;
}

static bool returnToYard(Train& t) {
	RailSim& R = g_rs;
	if (t.onboard) return false;
	int bj = -1, bs = -1;
	for (int j = NYT - 1; j >= 0 && bj < 0; j--) for (int k = YCAP - 1; k >= 0; k--) {
		bool taken = (R.yardOcc[t.line][j][k] >= 0);
		for (const Train& o : R.tr) if (o.id != t.id && o.st == TS_TOYARD && o.line == t.line && o.yTrk == j && o.ySlot == k) taken = true;
		if (!taken) { bj = j; bs = k; break; }
	}
	if (bj < 0) return false;
	const int side = g_yard[t.line].side;
	t.path = yardPath(t.line, bj); t.yTrk = bj; t.ySlot = bs;
	t.dir = -side; t.s = wrapS(slotHeadInto(t.line, bj, bs) - (float)side * TRAIN_LEN); t.v = 0.0f;
	t.st = TS_YARD; t.trip = -1; t.pos = 0; t.dwStage = 0; t.dwTimer = 0.0f; t.stuckT = 0.0f;
	t.doorL = t.doorR = 0.0f; t.tgtL = t.tgtR = false;
	R.yardOcc[t.line][bj][bs] = t.id;
	g_railRecoverCount++;
	return true;
}

static void trainMove(Train& t, float dt) {
	RailSim& R = g_rs;
	const bool shunt = (t.st == TS_TOYARD);
	float vAllow = shunt ? V_SHUNT : (t.st == TS_PULL ? V_PULL : V_MAX);
	auto cap = [&](float d, float vt, float margin) {
		const float dd = std::max(0.0f, d - margin);
		vAllow = std::min(vAllow, sqrtf(vt * vt + 2.0f * A_PAT * dd));
		};
	const float tgt = stopTargetHead(t);
	const float dStop = aheadDist(t, tgt);
	cap(dStop, 0.0f, 0.0f);
	if (t.st == TS_RUN) {
		const std::vector<float>& sg = g_sig[trainBase(t)]; const int n = (int)sg.size();
		for (int k = 0; k < n; k++) {
			const float d = aheadDist(t, sg[k]);
			if (d < 0.5f || d > 900.0f) continue;
			const int a = R.aspect[trainBase(t)][k];
			if (a == 0) cap(d, 0.0f, 9.0f);
			else if (a == 1) cap(d, 55.0f / 3.6f, 0.0f);
			else if (a == 2) cap(d, 75.0f / 3.6f, 0.0f);
		}
	}
	if (t.st == TS_RUN && yardWallActive(t)) {
		const float d = aheadDist(t, yardWallS(t.line));
		if (d > 0.5f && d < 1500.0f) cap(d, 0.0f, 16.0f);
	}
	{
		const float m = shunt ? 6.0f : 14.0f;
		float rl = 250.0f, rs = (t.dir > 0) ? t.s : t.s - rl;
		float mrs = rs, mrl = rl; clipMainZone(t.path, mrs, mrl);
		for (const Train& o : R.tr) {
			if (o.id == t.id) continue;
			float o0, ol;
			if (o.path == t.path) { bodyInterval(o, o0, ol); if (ol <= 0.0f || !ivOverlap(rs, rl, o0, ol, 0.0f)) continue; }
			else if (trainBase(o) == trainBase(t)) { mainInterval(o, o0, ol); if (ol <= 0.0f || mrl <= 0.0f || !ivOverlap(mrs, mrl, o0, ol, 0.0f)) continue; }
			else continue;
			const float nearEnd = (t.dir > 0) ? o0 : o0 + ol;
			const float farS = (t.dir > 0) ? o0 + ol : o0;
			float d = (float)t.dir * relS(nearEnd - t.s);
			const float df = (float)t.dir * relS(farS - t.s);
			if (df < 0.0f && d < 0.0f) continue;
			if (d < 0.0f) d = 0.0f;
			cap(d, 0.0f, m);
		}
	}
	const float v0 = t.v;
	if (t.v < vAllow) t.v = std::min(vAllow, t.v + accelAt(std::max(t.v, 1.0f)) * dt);
	else t.v = std::max(vAllow, t.v - A_BRK * dt);
	t.acc = (t.v - v0) / std::max(dt, 1e-4f);
	t.s = wrapS(t.s + (float)t.dir * t.v * dt);
	const float d2 = aheadDist(t, tgt);
	if ((d2 <= 0.30f && t.v < 2.0f) || d2 < -0.3f) { t.s = wrapS(tgt); onArrive(t); }
}

static void crossingUpdate(float dt) {
	RailSim& R = g_rs;
	for (Crossing& c : R.cx) {
		bool want = false;
		for (const Train& t : R.tr) {
			if (g_paths[t.path].yard >= 0 && t.st == TS_YARD) continue;
			if (t.v < 0.5f && t.st != TS_RUN) continue;
			const float d = (float)t.dir * relS(c.s - t.s);
			const float win = 30.0f + 8.0f * t.v;
			if (t.v > 0.8f && d > -(TRAIN_LEN + 8.0f) && d < win) { want = true; break; }
		}
		c.warn = false;
		switch (c.state) {
		case 0: if (want) { c.state = 1; c.timer = 0; } break;
		case 1: c.warn = true; c.timer += dt; if (c.timer >= 2.5f) c.state = 2; break;
		case 2: c.warn = true; c.arm = std::min(1.0f, c.arm + dt / 3.0f);
			if (!want) { c.timer += dt; if (c.timer >= 1.5f && c.arm >= 1.0f) c.state = 3; }
			else c.timer = 0; break;
		case 3: c.warn = true; c.arm = std::max(0.0f, c.arm - dt / 3.0f); if (want) c.state = 2; else if (c.arm <= 0.0f) c.state = 0; break;
		}
	}
}

static void dispatchYard() {
	RailSim& R = g_rs;
	for (int l = 0; l < NLINE; l++) {
		const int k = R.nextPull[l];
		if (k >= (int)R.trips[l].size() || k >= R.fleet[l]) continue;
		const double T = R.day * 86400.0 + R.trips[l][k];
		if (R.clock < T - (R.pullReal[l] + 10.0f) * RAIL_GPR) continue;
		Train* pick = nullptr; int pj = -1, pk = 99;
		for (int j = 0; j < NYT; j++) for (int s = 0; s < YCAP; s++) {
			const int id = R.yardOcc[l][j][s];
			if (id >= 0 && s < pk) { pk = s; pj = j; pick = &R.tr[id]; }
		}
		if (!pick) { if (R.clock > T + 120.0 * RAIL_GPR) R.nextPull[l]++; continue; }
		{
			const int trk = LINE_TRK[l]; const float dir = (float)TRK_DIR[trk];
			const float home = (dir > 0) ? g_stations[0].platformS0 - 30.0f : g_stations[0].platformS1 + 30.0f;
			float bs, bl; blockRange(trk, blockIndexAt(trk, wrapS(home - dir * 1.0f)), bs, bl);
			const float xs = bs, xl = bl;
			if (dir > 0) { bs -= 450.0f; bl += 450.0f; }
			else bl += 450.0f;
			bool busy = false;
			for (const Train& o : R.tr) {
				if (o.id == pick->id || trainBase(o) != trk) continue;
				float o0, ol; mainInterval(o, o0, ol);
				if (ol > 0.0f && (ivOverlap(xs, xl, o0, ol, 0.0f) || (o.v > 3.0f && ivOverlap(bs, bl, o0, ol, 0.0f)))) { busy = true; break; }
				if (o.st == TS_TERM || o.st == TS_TOYARD || o.st == TS_PULL) { busy = true; break; }
			}
			if (busy) continue;
		}
		R.yardOcc[l][pj][pk] = -1;
		pick->trip = k; R.nextPull[l]++; pick->depClock = R.day * 86400.0 + R.trips[l][k];
		pick->st = TS_PULL; pick->path = yardPath(l, pj); pick->yTrk = pj; pick->ySlot = pk;
		pick->dwStage = 0; pick->dwTimer = 0;
	}
}

static void railStep(float dt) {
	RailSim& R = g_rs;
	R.clock += dt * RAIL_GPR; R.simT += dt;
	{
		const int cd = (int)(R.clock / 86400.0);
		if (R.checkDay != cd && fmod(R.clock, 86400.0) >= FIRST_DEP_G - 3600.0) {
			R.checkDay = cd;
			for (Train& t : R.tr) if (t.st != TS_YARD) returnToYard(t);
		}
	}
	computeSignals();
	dispatchYard();
	for (Train& t : R.tr) {
		t.tripT += dt;
		switch (t.st) {
		case TS_YARD: t.v = 0; break;
		case TS_PULL: case TS_RUN: case TS_TOYARD: trainMove(t, dt); break;
		case TS_DWELL:
			if (dwellCycle(t, dt, R.clock, false)) { t.depAt[t.pos] = t.tripT; t.pos++; t.st = TS_RUN; t.dwStage = 0; t.dwTimer = 0; }
			break;
		case TS_ORIGIN:
			if (dwellCycle(t, dt, R.clock, true) && R.clock >= t.depClock) {
				t.trip = t.trip; t.st = TS_RUN; t.pos = 0; t.tripT = 0.0f; t.dwStage = 0; t.dwTimer = 0;
			}
			break;
		case TS_TERM:
			if (dwellCycle(t, dt, R.clock, false)) {
				if (!yardMoveClear(t)) break;
				int bj = -1;
				for (int j = NYT - 1; j >= 0 && bj < 0; j--) for (int s = YCAP - 1; s >= 0; s--) {
					bool taken = (R.yardOcc[t.line][j][s] >= 0);
					for (const Train& o : R.tr) if (o.st == TS_TOYARD && o.line == t.line && o.yTrk == j && o.ySlot == s) taken = true;
					if (!taken) { bj = j; t.yTrk = j; t.ySlot = s; break; }
				}
				if (bj >= 0) { flipTrain(t); t.st = TS_TOYARD; t.path = yardPath(t.line, bj); t.dwStage = 0; t.doorL = t.doorR = 0; }
			}
			break;
		}
		stepDoors(t, dt);

		if (t.st == TS_RUN || t.st == TS_PULL || t.st == TS_TOYARD) { if (t.v < 0.05f) t.stuckT += dt; else t.stuckT = 0.0f; }
		else t.stuckT = 0.0f;
		{
			const int homeTrk = LINE_TRK[t.line];
			const bool wrongPlace = (trainBase(t) != homeTrk) ||
				((t.st == TS_RUN || t.st == TS_PULL || t.st == TS_DWELL || t.st == TS_ORIGIN) && t.dir != TRK_DIR[homeTrk]);
			bool recover = false;
			if (t.st == TS_TOYARD) recover = t.stuckT > STUCK_SEC;
			else if (t.st == TS_TERM) recover = (t.dwStage == 5 && t.dwTimer > STUCK_SEC);
			else if (t.st == TS_RUN || t.st == TS_PULL) recover = wrongPlace ? (t.stuckT > 10.0f) : (t.stuckT > STUCK_SEC * 2.5f);
			if (recover) returnToYard(t);
		}
	}
	crossingUpdate(dt);
}


static void railClearState(float startSec) {
	RailSim& R = g_rs;
	R.tr.clear(); R.nextPull[0] = R.nextPull[1] = 0; R.nextTrip[0] = R.fleet[0]; R.nextTrip[1] = R.fleet[1]; R.platRes[0] = R.platRes[1] = -1;
	for (int l = 0; l < NLINE; l++) for (int j = 0; j < NYT; j++) for (int k = 0; k < YCAP; k++) R.yardOcc[l][j][k] = -1;
	for (Crossing& c : R.cx) { c.arm = 0; c.state = 0; c.timer = 0; c.warn = false; }
	memset(R.aspect, 3, sizeof(R.aspect)); memset(R.blockOcc, 0, sizeof(R.blockOcc));
	R.clock = R.day * 86400.0 + startSec; R.simT = 0; R.checkDay = -1;
}
static Train& addYardTrain(int line, int i) {
	RailSim& R = g_rs;
	Train t; t.id = (int)R.tr.size(); t.line = line; t.type = TT_LOCAL;
	const int j = NYT - 1 - i / YCAP, k = YCAP - 1 - (i % YCAP);
	const int side = g_yard[line].side;
	t.dir = -side; t.s = wrapS(slotHeadInto(line, j, k) - (float)side * TRAIN_LEN); t.v = 0;
	t.path = yardPath(line, j); t.st = TS_YARD; t.yTrk = j; t.ySlot = k;
	R.yardOcc[line][j][k] = t.id;
	R.tr.push_back(t);
	return R.tr.back();
}
static void genTrips() {
	RailSim& R = g_rs;
	for (int l = 0; l < NLINE; l++) {
		R.trips[l].clear();
		for (int k = 0; ; k++) {
			const double T = FIRST_DEP_G + k * HEADWAY_G;
			if (T + R.ti[l].run * RAIL_GPR > LAST_ARR_G) break;
			R.trips[l].push_back(T);
		}
	}
}
static bool refRun(int line, bool measurePull) {
	RailSim saved = g_rs; RailSim& R = g_rs;
	R.day = 0; railClearState(0.0f); R.checkDay = 0;
	for (int l = 0; l < NLINE; l++) R.trips[l].clear();
	bool ok = false;
	if (!measurePull) {
		Train t; t.id = 0; t.line = line; t.dir = TRK_DIR[LINE_TRK[line]]; t.s = stopHead(line, 0); t.path = LINE_TRK[line];
		t.st = TS_ORIGIN; t.depClock = 20.0 * RAIL_GPR; R.tr.push_back(t);
		for (int i = 0; i < 100000 && R.tr[0].st != TS_TERM; i++) railStep(0.05f);
		const Train& r = R.tr[0];
		if (r.st == TS_TERM) {
			ok = true;
			saved.ti[line].run = r.arrAt[g_nStops - 1];
			for (int p = 0; p < g_nStops; p++) { saved.ti[line].arr[p] = r.arrAt[p]; saved.ti[line].dep[p] = r.depAt[p]; }
		}
	}
	else {
		Train& t = addYardTrain(line, 0);
		t.st = TS_PULL; R.yardOcc[line][t.yTrk][t.ySlot] = -1; t.trip = 0;
		const int sd = TRK_DIR[LINE_TRK[line]]; (void)sd;
		t.depClock = 1e12;
		const double c0 = R.simT;
		for (int i = 0; i < 100000 && R.tr[0].st != TS_ORIGIN; i++) railStep(0.05f);
		if (R.tr[0].st == TS_ORIGIN) { ok = true; saved.pullReal[line] = (float)(R.simT - c0); }
	}
	g_rs = saved;
	return ok;
}
static void railReset(double todHours) {
	RailSim& R = g_rs;
	railClearState((float)std::min(todHours * 3600.0, 4.0 * 3600.0));
	for (int l = 0; l < NLINE; l++) for (int i = 0; i < R.fleet[l]; i++) addYardTrain(l, i);
}
static double g_railStepMax = 0.08;
static void railAdvanceTo(double target) {
	RailSim& R = g_rs;
	int guard = 0;
	while (R.clock < target - 1e-6 && guard++ < 400000) {
		const double dg = std::min(target - R.clock, g_railStepMax * RAIL_GPR);
		railStep((float)(dg / RAIL_GPR));
	}
	if (R.clock < target) R.clock = target;
}
static double g_railLastTod = -1.0;
static void railSync(double tod) {
	RailSim& R = g_rs;
	if (g_railLastTod < 0.0) { railReset(tod); g_railLastTod = tod; }
	if (tod < g_railLastTod - 12.0) { R.day++; R.nextPull[0] = R.nextPull[1] = 0; R.nextTrip[0] = R.fleet[0]; R.nextTrip[1] = R.fleet[1]; }
	g_railLastTod = tod;
	const double desired = R.day * 86400.0 + tod * 3600.0;
	if (desired < R.clock - 2.0 || desired - R.clock > 7.0 * 3600.0) { railReset(tod); }
	railAdvanceTo(desired);
}


static int stopPos(int line, int stn) { for (int p = 0; p < g_nStops; p++) if (g_order[line][p] == stn) return p; return -1; }
static void tripTimes(int line, int k, int stn, double& arr, double& dep) {
	const RailSim& R = g_rs; const double T = R.trips[line][k];
	if (stn == 0) { arr = T - 20.0 * RAIL_GPR; dep = T; return; }
	const int p = stopPos(line, stn);
	arr = T + R.ti[line].arr[p] * RAIL_GPR; dep = T + R.ti[line].dep[p] * RAIL_GPR;
}
struct BoardRow { int kind = 0; int trip = -1; double dep = 0; int type = 0; };
static void boardRows(int line, int stn, BoardRow rows[4]) {
	const RailSim& R = g_rs;
	for (int i = 0; i < 4; i++) rows[i] = BoardRow();
	const double now = fmod(R.clock, 86400.0);
	int stoppedTrip = -2;
	for (const Train& t : R.tr) {
		if (t.line != line || t.v > 0.01f) continue;
		if (t.st != TS_DWELL && t.st != TS_ORIGIN && t.st != TS_TERM) continue;
		if (curStop(t) != stn) continue;
		rows[0].type = t.type; stoppedTrip = t.trip;
		if (t.st == TS_TERM) { rows[0].kind = 3; }
		else { rows[0].kind = 1; rows[0].trip = t.trip; double a, d; if (t.trip >= 0) { tripTimes(line, t.trip, stn, a, d); rows[0].dep = d; } }
		break;
	}
	int n = 1;
	for (int k = 0; k < (int)R.trips[line].size() && n < 4; k++) {
		double a, d; tripTimes(line, k, stn, a, d);
		if (d <= now - 20.0 || k == stoppedTrip) continue;
		rows[n].kind = 2; rows[n].trip = k; rows[n].dep = d; rows[n].type = TT_LOCAL; n++;
	}
}

static int g_railCalIters = 2;
static void railInit() {
	RailSim& R = g_rs;
	R.cx.clear();
	for (float s : g_crossS) { Crossing c; c.s = s; R.cx.push_back(c); }
	const float Hreal = (float)(HEADWAY_G / RAIL_GPR);
	const float DWELL_MIN = 11.0f, LAYOVER_MIN = 14.0f;
	for (int l = 0; l < NLINE; l++) {
		R.extraDwell[l] = 0.0f;
		refRun(l, false);
		const float runE = R.ti[l].run + 32.0f;
		const float phase = fmodf(runE, Hreal), lo = 5.0f, hi = Hreal - LAYOVER_MIN;
		if (phase < lo || phase > hi) {
			R.extraDwell[l] = fmodf(((lo + hi) * 0.5f - phase) + Hreal * 2.0f, Hreal);
			refRun(l, false);
		}
		refRun(l, true);
		R.fleet[l] = std::min(NYT * YCAP, (int)ceil((R.ti[l].run + 32.0f + DWELL_MIN + 4.0f) / Hreal));
	}
	genTrips();

	for (int it = 0; it < g_railCalIters; it++) {
		RailSim& Q = g_rs;
		for (int l = 0; l < NLINE; l++) { Q.calN[l] = 0; for (int p = 0; p < 8; p++) Q.calArr[l][p] = Q.calDep[l][p] = 0; }
		const double saveStep = g_railStepMax; g_railStepMax = 0.2;
		Q.day = 0; railReset(0.0); railAdvanceTo(15.0 * 3600.0);
		g_railStepMax = saveStep;
		bool again = false;
		for (int l = 0; l < NLINE; l++) {
			if (Q.calN[l] < 2) continue;
			TimeInfo ni;
			for (int p = 0; p < g_nStops; p++) { ni.arr[p] = (float)(Q.calArr[l][p] / Q.calN[l]); ni.dep[p] = (float)(Q.calDep[l][p] / Q.calN[l]); }
			ni.run = ni.arr[g_nStops - 1];
			Q.ti[l] = ni;
			const int need = std::min(NYT * YCAP, (int)ceil((ni.run + LAYOVER_MIN + 3.0f) / Hreal));
			if (need > Q.fleet[l]) { Q.fleet[l] = need; again = true; }
		}
		genTrips();
		if (!again) break;
	}
}


static void addBeam(Verts& v, V3 a, V3 b, float hw, float hh, V3 col) {
	V3 d = b - a; const float L = sqrtf(dot(d, d)); if (L < 1e-5f) return;
	const V3 f = d * (1.0f / L), ref = (fabsf(f.y) > 0.95f) ? V3{ 1, 0, 0 } : V3{ 0, 1, 0 };
	const V3 r = norm(cross(ref, f)), u = cross(f, r);
	addBox(v, (a + b) * 0.5f, r, u, f, { hw, hh, L * 0.5f }, col);
}
static V3 fPt(const TFrame& F, float lat, float up, float fwd = 0.0f) { return F.p + F.r * lat + F.u * up + F.f * fwd; }
static void fBox(Verts& v, const TFrame& F, float lat, float up, float fwd, V3 half, V3 col) { addBox(v, fPt(F, lat, up, fwd), F.r, F.u, F.f, half, col); }
static void addSleeper(Verts& v, const TFrame& F, float lat, V3 col) {
	const float hx = 1.2f, hy = 0.05f, hz = 0.16f, cy = 0.28f;
	const V3 c = F.p + F.r * lat + F.u * cy;
	auto quad = [&](V3 a, V3 b, V3 cc, V3 d, V3 n) { pushV(v, a, n, col); pushV(v, b, n, col); pushV(v, cc, n, col); pushV(v, a, n, col); pushV(v, cc, n, col); pushV(v, d, n, col); };
	quad(c + F.u * hy - F.r * hx - F.f * hz, c + F.u * hy + F.r * hx - F.f * hz, c + F.u * hy + F.r * hx + F.f * hz, c + F.u * hy - F.r * hx + F.f * hz, F.u);
	for (int sg = -1; sg <= 1; sg += 2) {
		const V3 o = F.f * (hz * sg);
		quad(c + o - F.r * hx - F.u * hy, c + o + F.r * hx - F.u * hy, c + o + F.r * hx + F.u * hy, c + o - F.r * hx + F.u * hy, F.f * (float)sg);
	}
}

struct StopMarker { V3 pos; int stn, line; };
static std::vector<std::pair<V3, V3>> g_noColBoxes;
struct BoardObj { V3 c, r, f, u; int stn, line; float w, h; };
struct SigObj { int trk, k; V3 head, face; };
struct XingObj { float s; TFrame F; };
static std::vector<StopMarker> g_markers;
static std::vector<BoardObj> g_boardObjs;
static std::vector<SigObj> g_sigObjs;
static std::vector<XingObj> g_xingObjs;
static std::vector<TFrame> g_benches;
static std::vector<float> g_fenceL[2];
static const float FENCE_DS = 3.5f;
static int g_fenceN = 0;

static float fenceEnv(int side, float s) {
	const float q = wrapS(s) / FENCE_DS; const int i0 = (int)q, i1 = (i0 + 1) % g_fenceN; const float t = q - i0;
	return g_fenceL[side][i0 % g_fenceN] * (1.0f - t) + g_fenceL[side][i1] * t;
}
struct PfDef { float a, b; int nf; int trk[2]; float edge[2]; int inward[2]; };
static int stationPlatforms(int stn, PfDef* out) {
	if (stn == 0) {
		const float a0 = TRK_BASE[0] - BIG_SHIFT + PF_EDGE, b0 = TRK_BASE[1] - PF_EDGE, a1 = TRK_BASE[2] + PF_EDGE, b1 = TRK_BASE[3] + BIG_SHIFT - PF_EDGE;
		out[0] = { a0, b0, 2, { 0, 1 }, { a0, b0 }, { 1, -1 } };
		out[1] = { a1, b1, 2, { 2, 3 }, { a1, b1 }, { 1, -1 } };
		return 2;
	}
	out[0] = { TRK_BASE[0] - PF_EDGE - SIDE_W, TRK_BASE[0] - PF_EDGE, 1, { 0, 0 }, { TRK_BASE[0] - PF_EDGE, 0 }, { -1, 0 } };
	out[1] = { TRK_BASE[3] + PF_EDGE, TRK_BASE[3] + PF_EDGE + SIDE_W, 1, { 3, 3 }, { TRK_BASE[3] + PF_EDGE, 0 }, { 1, 0 } };
	return 2;
}
static float bridgeS(const StationData& st) { return st.platformS0 + 6.0f; }
static bool inLeadZone(float s) {
	for (int l = 0; l < NLINE; l++) {
		const YardDef& Y = g_yard[l];
		const float a = Y.b[NYT - 1], b = Y.b[0] + Y.side * YARD_LEN;
		const float lo = std::min(a, b) - 6.0f, len = fabsf(relS(b - a)) + 12.0f;
		const float d = relS(s - lo); if (d > 0 && d < len) return true;
	}
	return false;
}

static const std::vector<PPt> PF_BALLAST = { { -2.8f, -0.35f }, { 2.8f, -0.35f }, { 2.0f, 0.05f }, { 1.5f, 0.25f }, { -1.5f, 0.25f }, { -2.0f, 0.05f } };
static void buildBed(Verts& v, int pid, float s0, float s1) {
	if (s1 < s0) std::swap(s0, s1);
	const V3 b0 = { 0.50f, 0.48f, 0.45f };
	const std::vector<V3> ecB = { b0 * 0.6f, b0 * 0.9f, b0 * 0.95f, b0, b0 * 0.95f, b0 * 0.9f };
	sweepProfileF(v, PF_BALLAST, ecB, s0, s1, 4.0f, [pid](float s) { return pathLat(pid, s); }, 0.0f, true, b0);
	const std::vector<PPt> pf = {
		{ -0.10f, 0.32f }, { 0.10f, 0.32f }, { 0.03f, 0.40f }, { 0.03f, 0.47f }, { 0.07f, 0.52f }, { -0.07f, 0.52f }, { -0.03f, 0.47f }, { -0.03f, 0.40f } };
	const V3 side = { 0.46f, 0.36f, 0.30f }, web = { 0.55f, 0.50f, 0.47f }, top = { 0.80f, 0.80f, 0.83f }, dark = { 0.30f, 0.28f, 0.27f };
	const std::vector<V3> ec = { dark, side, web, side, top, side, web, side };
	for (int sg = -1; sg <= 1; sg += 2) sweepProfileF(v, pf, ec, s0, s1, 6.0f, [pid, sg](float s) { return pathLat(pid, s) + 0.7175f * sg; }, 0.0f, false, web);
	for (float s = s0; s < s1; s += 0.85f) {
		const TFrame F = trackFrame(s);
		const float k = 0.88f + 0.24f * (0.5f + 0.5f * sinf(s * 12.9898f));
		addSleeper(v, F, pathLat(pid, s), V3{ 0.36f, 0.26f, 0.18f } * k);
	}
}
static void addStair(Verts& v, float s0, float lat, float halfW, float yTop, float yBot, float groundY, float tread, float dir, V3 col) {

	const float rise = 0.1854f * 1.0f;
	const int n = std::max(1, (int)ceilf((yTop - yBot) / rise));
	const float rr = (yTop - yBot) / n;
	for (int i = 0; i < n; i++) {
		const float top = yTop - (i + 1) * rr;
		const float sc = s0 + dir * (i + 0.5f) * tread;
		const TFrame F = trackFrame(sc);
		const float y0 = groundY - 0.5f, y1 = top;
		if (y1 <= y0 + 0.02f) break;
		fBox(v, F, lat, (y0 + y1) * 0.5f, 0.0f, { halfW, (y1 - y0) * 0.5f, tread * 0.5f }, col * (0.92f + 0.08f * (i & 1)));
	}
}
static void addFenceSpan(Verts& v, V3 a, V3 b, float baseY, V3 col) {
	const float ys[3] = { 0.40f, 0.85f, 1.30f };
	for (int k = 0; k < 3; k++) addBeam(v, a + V3{ 0, ys[k] + baseY, 0 }, b + V3{ 0, ys[k] + baseY, 0 }, 0.022f, 0.022f, col);
}
static void addPost(Verts& v, V3 p, float h, V3 col) { addBox(v, p + V3{ 0, h * 0.5f, 0 }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.04f, h * 0.5f, 0.04f }, col); }

static void buildStation(Verts& v, int stn) {
	const StationData& st = g_stations[stn];
	PfDef pd[2]; stationPlatforms(stn, pd);
	const V3 cc = { 0.72f, 0.72f, 0.70f }, steel = { 0.58f, 0.60f, 0.62f }, rail = { 0.30f, 0.32f, 0.34f }, roofc = { 0.82f, 0.84f, 0.86f };
	const float P0 = st.platformS0, P1 = st.platformS1;
	for (int pi = 0; pi < 2; pi++) {
		const PfDef& P = pd[pi];
		g_curE = 1.0f;
		{
			const std::vector<PPt> pf = { { P.a, -0.5f }, { P.b, -0.5f }, { P.b, 1.63f }, { P.b - 0.06f, 1.67f }, { P.a + 0.06f, 1.67f }, { P.a, 1.63f } };
			const std::vector<V3> ec = { cc * 0.6f, cc * 0.85f, cc * 0.95f, cc * 1.04f, cc * 0.95f, cc * 0.85f };
			sweepProfileF(v, pf, ec, P0, P1, 2.0f, [](float) { return 0.0f; }, 0.0f, true, cc * 0.9f);
		}
		for (int f = 0; f < P.nf; f++) {
			const float e = P.edge[f], in = (float)P.inward[f];
			const float x0 = e + in * 0.05f, x1 = e + in * 0.45f;
			const std::vector<PPt> pf = { { std::min(x0, x1), 1.665f }, { std::max(x0, x1), 1.665f }, { std::max(x0, x1), 1.70f }, { std::min(x0, x1), 1.70f } };
			const V3 y0 = { 0.95f, 0.85f, 0.10f };
			const std::vector<V3> ec = { y0 * 0.7f, y0 * 0.85f, y0, y0 * 0.85f };
			sweepProfileF(v, pf, ec, P0, P1, 2.0f, [](float) { return 0.0f; }, 0.0f, true, y0);
		}
		g_curE = 0.0f;

		for (int f = 0; f < P.nf; f++) {
			const int trk = P.trk[f]; const float lat = P.edge[f] + (float)P.inward[f] * 0.20f;
			std::vector<std::pair<float, float>> gaps;
			int line = -1; for (int l = 0; l < NLINE; l++) if (LINE_TRK[l] == trk) line = l;
			if (line >= 0) {
				const int dir = TRK_DIR[trk]; const float sh = stopHead(line, stn);
				for (int c = 0; c < NUM_CARS; c++) {
					const float sc = sh - dir * (CAR_LEN * 0.5f + c * (CAR_LEN + CAR_GAP));
					for (int k = 0; k < 3; k++) { const float sd = sc + dir * DOOR_Z[k]; gaps.push_back({ sd - 1.05f, sd + 1.05f }); }
				}
				std::sort(gaps.begin(), gaps.end());
			}
			float cur = P0 + 0.3f;
			auto emit = [&](float a, float b) {
				if (b - a < 0.05f) return;
				const int n = std::max(1, (int)ceilf((b - a) / 2.5f)); const float step = (b - a) / n;
				for (int i = 0; i <= n; i++) { const TFrame F = trackFrame(a + i * step); addPost(v, fPt(F, lat, PF_TOP), 1.25f, steel); }
				for (int i = 0; i < n; i++) {
					const TFrame F0 = trackFrame(a + i * step), F1 = trackFrame(a + (i + 1) * step);
					addFenceSpan(v, fPt(F0, lat, PF_TOP), fPt(F1, lat, PF_TOP), 0.0f, rail);
				}
				};
			for (auto& g : gaps) { if (g.second < cur || g.first > P1 - 0.3f) continue; emit(cur, std::min(g.first, P1 - 0.3f)); cur = std::max(cur, g.second); }
			emit(cur, P1 - 0.3f);
		}
		if (stn != 0) {
			const float lat = (P.inward[0] < 0) ? P.a + 0.15f : P.b - 0.15f;
			const int n = (int)ceilf((P1 - P0) / 2.5f);
			for (int i = 0; i <= n; i++) { const TFrame F = trackFrame(P0 + (P1 - P0) * i / n); addPost(v, fPt(F, lat, PF_TOP), 1.25f, steel); }
			for (int i = 0; i < n; i++) addFenceSpan(v, fPt(trackFrame(P0 + (P1 - P0) * i / n), lat, PF_TOP), fPt(trackFrame(P0 + (P1 - P0) * (i + 1) / n), lat, PF_TOP), 0.0f, rail);
		}
		for (int e = 0; e < 2; e++) {
			const TFrame F = trackFrame(e ? P1 - 0.3f : P0 + 0.3f);
			for (int k = 0; k < 3; k++) { const float ys[3] = { 0.40f, 0.85f, 1.30f }; fBox(v, F, (P.a + P.b) * 0.5f, PF_TOP + ys[k], 0.0f, { (P.b - P.a) * 0.5f, 0.022f, 0.022f }, rail); }
			for (int i = 0; i <= 4; i++) addPost(v, fPt(F, P.a + (P.b - P.a) * i / 4.0f, PF_TOP), 1.25f, steel);
		}

		const float R0 = P0 + 30.0f;
		{
			const std::vector<PPt> pf = { { P.a - 0.4f, 5.90f }, { P.b + 0.4f, 5.90f }, { P.b + 0.4f, 6.12f }, { P.a - 0.4f, 6.12f } };
			const std::vector<V3> ec = { roofc * 0.7f, roofc * 0.8f, roofc, roofc * 0.8f };
			sweepProfileF(v, pf, ec, R0, P1, 6.0f, [](float) { return 0.0f; }, 0.0f, true, roofc * 0.8f);
		}
		const float postLat = (stn == 0) ? (P.a + P.b) * 0.5f : ((P.inward[0] < 0) ? P.a + 0.5f : P.b - 0.5f);
		for (float s = R0 + 6.0f; s < P1 - 3.0f; s += 12.0f) {
			const TFrame F = trackFrame(s);
			fBox(v, F, postLat, (PF_TOP + 5.9f) * 0.5f, 0.0f, { 0.14f, (5.9f - PF_TOP) * 0.5f, 0.14f }, steel);
			g_curE = 1.25f; fBox(v, F, (P.a + P.b) * 0.5f, 5.80f, 6.0f, { 0.12f, 0.04f, 0.9f }, { 0.95f, 0.95f, 0.85f }); g_curE = 0.0f;
			if (((int)((s - R0) / 12.0f)) % 2 == 0) g_lampPos.push_back(fPt(F, (P.a + P.b) * 0.5f, 5.6f, 6.0f));
		}

		for (int f = 0; f < P.nf; f++) {
			const int trk = P.trk[f]; int line = -1; for (int l = 0; l < NLINE; l++) if (LINE_TRK[l] == trk) line = l;
			if (line < 0) continue;
			const float e = P.edge[f], in = (float)P.inward[f];
			for (int bI = 0; bI < 2; bI++) {
				const float sb = P0 + (bI ? 100.0f : 56.0f);
				const TFrame F = trackFrame(sb);
				const float lc = e + in * 2.5f;
				fBox(v, F, lc, 4.95f, 0.0f, { 2.45f, 0.95f, 0.09f }, { 0.07f, 0.07f, 0.08f });
				for (int k = -1; k <= 1; k += 2) fBox(v, F, lc + k * 2.0f, 5.55f, 0.0f, { 0.02f, 0.35f, 0.02f }, steel);
				BoardObj B; B.c = fPt(F, lc, 4.95f); B.r = F.r; B.f = F.f; B.u = F.u; B.stn = stn; B.line = line; B.w = 4.8f; B.h = 1.8f;
				g_boardObjs.push_back(B);
			}
			{
				const TFrame F = trackFrame(stopHead(line, stn));
				const float lc = e + in * 0.75f;
				fBox(v, F, lc, PF_TOP + 0.5f, 0.0f, { 0.06f, 0.5f, 0.06f }, { 0.3f, 0.3f, 0.3f });
				fBox(v, F, lc, PF_TOP + 1.6f, 0.0f, { 0.05f, 0.4f, 0.5f }, { 0.9f, 0.1f, 0.1f });
				g_markers.push_back({ fPt(F, lc, PF_TOP + 1.6f), stn, line });
			}
			for (int bI = 0; bI < 2; bI++) {
				const float s = P0 + (bI ? 118.0f : 76.0f) - 6.0f;
				const TFrame F = trackFrame(s);
				const float lat = e + in * 1.9f;
				TFrame B;
				if (in < 0) { B = F; B.p = F.p + F.r * (lat + 4.1f); }
				else { B.p = F.p + F.r * (lat - 4.1f); B.r = F.r * -1.0f; B.u = F.u; B.f = F.f * -1.0f; }
				g_curE = 1.0f;
				addBox(v, B.p + B.r * (-4.1f) + B.u * 2.12f, B.r, B.u, B.f, { 0.28f, 0.04f, 0.80f }, { 0.45f, 0.30f, 0.18f });
				addBox(v, B.p + B.r * (-4.4f) + B.u * 2.52f, B.r, B.u, B.f, { 0.03f, 0.30f, 0.80f }, { 0.45f, 0.30f, 0.18f });
				for (int sg = -1; sg <= 1; sg += 2) addBox(v, B.p + B.r * (-4.1f) + B.u * 1.89f + B.f * (0.65f * sg), B.r, B.u, B.f, { 0.22f, 0.20f, 0.04f }, { 0.25f, 0.25f, 0.27f });
				g_curE = 0.0f;
				g_benches.push_back(B);
			}
		}
	}

	{
		const float bs = bridgeS(st);
		const float latL = -22.0f, latR = (stn == 0) ? pd[1].b + PF_EDGE + 3.0f : pd[1].b + 0.8f;   // 駅0は右端の線路の外側まで渡り廊下を延ばし、端の柱が電車に当たらないようにする
		const float H = 7.4f, GY = -0.2f;
		const TFrame F = trackFrame(bs);
		const V3 deckC = { 0.66f, 0.67f, 0.68f }, wallC = { 0.80f, 0.82f, 0.84f };
		g_curE = 1.0f;
		fBox(v, F, (latL + latR) * 0.5f, H - 0.25f, 0.0f, { (latR - latL) * 0.5f, 0.25f, 2.2f }, deckC);
		g_curE = 0.0f;
		fBox(v, F, (latL + latR) * 0.5f, 10.15f, 0.0f, { (latR - latL) * 0.5f + 0.4f, 0.10f, 2.7f }, roofc);

		fBox(v, F, (latL + latR) * 0.5f, H + 0.55f, -2.1f, { (latR - latL) * 0.5f, 0.55f, 0.05f }, wallC);
		std::vector<std::pair<float, float>> openings;
		openings.push_back({ latL, latL + 3.4f });
		for (int pi = 0; pi < 2; pi++) openings.push_back({ (pd[pi].a + pd[pi].b) * 0.5f - 1.2f, (pd[pi].a + pd[pi].b) * 0.5f + 1.2f });
		std::sort(openings.begin(), openings.end());
		float cur = latL;
		for (auto& o : openings) { if (o.first > cur) fBox(v, F, (cur + o.first) * 0.5f, H + 0.55f, 2.1f, { (o.first - cur) * 0.5f, 0.55f, 0.05f }, wallC); cur = std::max(cur, o.second); }
		if (latR > cur) fBox(v, F, (cur + latR) * 0.5f, H + 0.55f, 2.1f, { (latR - cur) * 0.5f, 0.55f, 0.05f }, wallC);
		fBox(v, F, latL, H + 0.55f, 0.0f, { 0.05f, 0.55f, 2.2f }, wallC);
		fBox(v, F, latR, H + 0.55f, 0.0f, { 0.05f, 0.55f, 2.2f }, wallC);
		for (float lt = latL + 0.3f; lt < latR; lt += 6.0f)
			for (int sg = -1; sg <= 1; sg += 2) fBox(v, F, lt, (H + 10.05f) * 0.5f, 2.1f * sg, { 0.07f, (10.05f - H) * 0.5f, 0.07f }, steel);

		const float cols[4] = { latL + 0.5f, (pd[0].a + pd[0].b) * 0.5f, (pd[1].a + pd[1].b) * 0.5f, latR - 0.4f };
		for (int i = 0; i < 4; i++) { const float y0 = (i == 1 || i == 2) ? PF_TOP : GY - 0.4f; fBox(v, F, cols[i], (y0 + H - 0.5f) * 0.5f, 0.0f, { 0.3f, (H - 0.5f - y0) * 0.5f, 0.3f }, deckC); }

		g_curE = 1.0f;
		const float sTop = bs + 2.2f;
		for (int pi = 0; pi < 2; pi++) addStair(v, sTop, (pd[pi].a + pd[pi].b) * 0.5f, 1.1f, H, PF_TOP, PF_TOP, 0.27f, 1.0f, { 0.68f, 0.68f, 0.66f });
		addStair(v, sTop, latL + 1.7f, 1.7f, H, GY, GY, 0.27f, 1.0f, { 0.68f, 0.68f, 0.66f });
		g_curE = 0.0f;

		const float sg0 = sTop + 41 * 0.27f + 0.4f;
		const TFrame G = trackFrame(sg0 + 0.6f);
		const float gl = latL + 1.7f;
		for (int k = -1; k <= 1; k++) {
			fBox(v, G, gl + k * 1.2f, 0.55f - 0.2f + 0.1f, 0.0f, { 0.20f, 0.55f, 0.60f }, { 0.78f, 0.80f, 0.82f });
			fBox(v, G, gl + k * 1.2f, 1.12f - 0.1f, 0.1f, { 0.21f, 0.03f, 0.45f }, { 0.12f, 0.20f, 0.55f });
		}
		g_curE = 2.0f;
		for (int k = 0; k < 2; k++) {
			fBox(v, G, gl + (k - 0.5f) * 1.2f, 1.45f, -0.55f, { 0.18f, 0.12f, 0.02f }, { 0.15f, 0.85f, 0.30f });
			V3 lo = { 1e9f, 1e9f, 1e9f }, hi = { -1e9f, -1e9f, -1e9f };
			for (int a = -1; a <= 1; a += 2) for (int b = -1; b <= 1; b += 2) for (int c = -1; c <= 1; c += 2) {
				const V3 q = fPt(G, gl + (k - 0.5f) * 1.2f + a * 0.18f, 1.45f + b * 0.12f, -0.55f + c * 0.02f);
				lo = { std::min(lo.x, q.x), std::min(lo.y, q.y), std::min(lo.z, q.z) }; hi = { std::max(hi.x, q.x), std::max(hi.y, q.y), std::max(hi.z, q.z) };
			}
			g_noColBoxes.push_back({ lo - V3{ 0.02f, 0.02f, 0.02f }, hi + V3{ 0.02f, 0.02f, 0.02f } });   // 改札の上の緑の表示板は当たり判定なし
		}
		g_curE = 0.0f;
		for (int sg = -1; sg <= 1; sg += 2) fBox(v, G, gl + sg * 1.78f, 0.45f, 0.0f, { 0.10f, 0.65f, 0.60f }, { 0.78f, 0.80f, 0.82f });

		const TFrame RF = trackFrame(sg0 + 2.0f);
		fBox(v, RF, gl, 3.35f, 0.0f, { 2.6f, 0.08f, 4.2f }, roofc);
		for (int sx = -1; sx <= 1; sx += 2) for (int sz = -1; sz <= 1; sz += 2) fBox(v, RF, gl + 2.45f * sx, 1.55f, 3.9f * sz, { 0.07f, 1.8f, 0.07f }, steel);
	}
}

static void buildSignals(Verts& v) {
	const V3 steel = { 0.40f, 0.42f, 0.44f }, hous = { 0.10f, 0.10f, 0.11f };
	for (int t = 0; t < NTRK; t++) {
		const float dir = (float)TRK_DIR[t];
		for (int k = 0; k < (int)g_sig[t].size(); k++) {
			const float s = g_sig[t][k];
			const TFrame F = trackFrame(s);
			const float pl = (t == 0) ? baseLat(0, s) - 2.3f : (t == 3) ? baseLat(3, s) + 2.3f : 0.0f;
			const float toward = (t == 0) ? 1.0f : (t == 3) ? -1.0f : ((dir > 0) ? 1.0f : -1.0f);
			const float hl = pl + toward * 0.95f;
			fBox(v, F, pl, 2.1f, 0.0f, { 0.08f, 2.4f, 0.08f }, steel);
			fBox(v, F, pl, -0.2f, 0.0f, { 0.30f, 0.30f, 0.30f }, { 0.55f, 0.55f, 0.53f });
			addBeam(v, fPt(F, pl, 4.3f), fPt(F, hl, 4.3f), 0.03f, 0.03f, steel);
			fBox(v, F, hl, 4.55f, 0.0f, { 0.21f, 0.63f, 0.12f }, hous);
			const V3 face = F.f * (-dir);
			for (int i = 0; i < 3; i++) addBox(v, fPt(F, hl, 4.55f + (1 - i) * 0.40f, -dir * 0.121f), F.r, F.u, F.f, { 0.15f, 0.15f, 0.01f }, { 0.04f, 0.04f, 0.045f });
			fBox(v, F, hl, 5.22f, -dir * 0.06f, { 0.23f, 0.03f, 0.19f }, hous);
			SigObj o; o.trk = t; o.k = k; o.head = fPt(F, hl, 4.55f); o.face = face; g_sigObjs.push_back(o);
		}
	}
}

static void buildCrossing(Verts& v, float sc) {
	const TFrame F = trackFrame(sc);
	const V3 deckC = { 0.60f, 0.60f, 0.58f }, road = { 0.20f, 0.20f, 0.22f }, steel = { 0.55f, 0.57f, 0.58f };
	g_curE = 0.0f;
	{
		const std::vector<PPt> pf = { { -11.4f, -0.5f }, { 11.4f, -0.5f }, { 11.4f, -0.17f }, { 8.4f, 0.47f }, { -8.4f, 0.47f }, { -11.4f, -0.17f } };
		const std::vector<V3> ec = { deckC * 0.6f, deckC * 0.9f, road * 1.2f, road * 1.4f, road * 1.2f, deckC * 0.9f };
		sweepProfileF(v, pf, ec, sc - 3.0f, sc + 3.0f, 3.0f, [](float) { return 0.0f; }, 0.0f, true, deckC);
	}
	for (int sg = -1; sg <= 1; sg += 2) {
		fBox(v, F, sg * (11.4f + 14.0f), -0.17f - 0.15f, 0.0f, { 14.0f, 0.15f, 3.0f }, road);
		const float lc = sg * 12.6f;
		for (int sz = -1; sz <= 1; sz += 2) {
			fBox(v, F, lc, 1.6f, 3.9f * sz, { 0.10f, 1.8f, 0.10f }, steel);
			fBox(v, F, lc, 0.0f, 3.9f * sz, { 0.30f, 0.25f, 0.30f }, { 0.55f, 0.55f, 0.53f });
			fBox(v, F, lc, 3.55f, 3.9f * sz, { 0.40f, 0.04f, 0.04f }, steel);
			fBox(v, F, lc, 3.0f, 3.9f * sz, { 0.05f, 0.30f, 0.30f }, { 0.95f, 0.95f, 0.30f });
			for (int k = -1; k <= 1; k += 2) fBox(v, F, lc + k * 0.32f, 3.55f, 3.9f * sz + 0.0f, { 0.12f, 0.12f, 0.12f }, { 0.08f, 0.08f, 0.09f });
		}
	}
	g_xingObjs.push_back({ sc, F });
}

static void buildFences(Verts& v) {
	const V3 col = { 0.38f, 0.42f, 0.40f };
	const int n = g_fenceN;
	for (int side = 0; side < 2; side++) {
		const float sg = side ? 1.0f : -1.0f;
		for (int i = 0; i < n; i++) {
			const float s0 = i * FENCE_DS, s1 = (i + 1) * FENCE_DS;
			bool skip = false;
			for (float cs : g_crossS) { const float d = fabsf(relS((s0 + s1) * 0.5f - cs)); if (d < 5.2f) { skip = true; break; } }
			if (skip) continue;
			const TFrame F0 = trackFrame(s0), F1 = trackFrame(s1);
			const V3 a = F0.p + F0.r * (sg * g_fenceL[side][i]) + V3{ 0, -0.2f, 0 }, b = F1.p + F1.r * (sg * g_fenceL[side][(i + 1) % n]) + V3{ 0, -0.2f, 0 };
			addPost(v, a, 1.45f, col);
			addFenceSpan(v, a, b, 0.0f, col);
		}
	}
}
static void computeFence() {
	const int n = (int)ceilf(g_totalTrackLen / FENCE_DS); g_fenceN = n;
	for (int side = 0; side < 2; side++) {
		std::vector<float>& e = g_fenceL[side]; e.assign(n, 0.0f);
		for (int i = 0; i < n; i++) {
			const float s = i * FENCE_DS; float m = 0.0f;
			for (int p = 0; p < (int)g_paths.size(); p++) {
				const float lat = pathLat(p, s);
				if (g_paths[p].yard >= 0) { const YardDef& Y = g_yard[g_paths[p].yard]; const float u = (float)Y.side * relS(s - Y.b[g_paths[p].j]); if (u < -8.0f || u > YARD_LEN + 6.0f + TRAIN_LEN + (YCAP - 1) * SLOT_PITCH + 14.0f) continue; }
				if ((side == 0 && lat < 0) || (side == 1 && lat > 0)) m = std::max(m, fabsf(lat) + 3.4f);
			}
			for (int stn = 0; stn < (int)g_stations.size(); stn++) {
				PfDef pd[2]; stationPlatforms(stn, pd);
				const StationData& st = g_stations[stn];
				if (fabsf(relS(s - (st.platformS0 + st.platformS1) * 0.5f)) < 70.0f) for (int pi = 0; pi < 2; pi++) {
					const float ex = (side == 0) ? -std::min(pd[pi].a, pd[pi].b) : std::max(pd[pi].a, pd[pi].b);
					if ((side == 0 && pd[pi].a < 0) || (side == 1 && pd[pi].b > 0)) m = std::max(m, ex + 0.8f);
				}
			}
			e[i] = std::max(m, 9.0f);
		}
		for (int it = 0; it < 3; it++) {
			for (int i = 0; i < n; i++) { const int j = (i + n - 1) % n; e[i] = std::max(e[i], e[j] - 0.5f * FENCE_DS); }
			for (int i = n - 1; i >= 0; i--) { const int j = (i + 1) % n; e[i] = std::max(e[i], e[j] - 0.5f * FENCE_DS); }
		}
	}
}

static Verts buildTrack() {
	Verts v;
	g_lampPos.clear(); g_markers.clear(); g_boardObjs.clear(); g_sigObjs.clear(); g_xingObjs.clear(); g_benches.clear();
	addAABB(v, { 0, -60.0f, 0 }, { 6000, 0.5f, 6000 }, { 0.32f, 0.52f, 0.28f });
	const float LEN = g_totalTrackLen;
	computeFence();
	for (int t = 0; t < NTRK; t++) buildBed(v, t, 0.0f, LEN);
	for (int l = 0; l < NLINE; l++) for (int j = 0; j < NYT; j++) {
		const YardDef& Y = g_yard[l];
		const float farS = Y.b[j] + Y.side * (YARD_LEN + 6.0f + TRAIN_LEN + (YCAP - 1) * SLOT_PITCH + 12.0f);
		buildBed(v, yardPath(l, j), Y.b[j] - Y.side * 6.0f, farS);
	}
	for (int i = 0; i < (int)g_stations.size(); i++) buildStation(v, i);
	for (float cs : g_crossS) buildCrossing(v, cs);
	buildSignals(v);
	buildFences(v);

	{
		const V3 steel = { 0.58f, 0.60f, 0.62f }, wire = { 0.12f, 0.12f, 0.13f };
		for (float s = 5.0f; s < LEN - 5.0f; s += 60.0f) {
			if (inLeadZone(s)) continue;
			bool isNear = false;
			for (const auto& st : g_stations) if (fabsf(relS(s - bridgeS(st))) < 7.0f) isNear = true;
			for (float cs : g_crossS) if (fabsf(relS(s - cs)) < 9.0f) isNear = true;
			if (isNear) continue;
			const TFrame F = trackFrame(s);
			const float eL = fenceEnv(0, s) + 0.9f, eR = fenceEnv(1, s) + 0.9f;
			for (int sd = 0; sd < 2; sd++) {
				const float lat = sd ? eR : -eL;
				addBox(v, F.p + F.r * lat + F.u * 3.9f, F.r, F.u, F.f, { 0.12f, 4.2f, 0.12f }, steel);
				addBox(v, F.p + F.r * lat + F.u * (-0.1f), F.r, F.u, F.f, { 0.30f, 0.35f, 0.30f }, { 0.55f, 0.55f, 0.53f });
			}
			addBox(v, F.p + F.r * ((eR - eL) * 0.5f) + F.u * 7.1f, F.r, F.u, F.f, { (eR + eL) * 0.5f, 0.08f, 0.10f }, steel);
			for (int t = 0; t < NTRK; t++) {
				const float lt = baseLat(t, s);
				addBox(v, F.p + F.r * lt + F.u * 6.8f, F.r, F.u, F.f, { 0.03f, 0.30f, 0.03f }, steel);
			}
		}
		const std::vector<PPt> wf = { { -0.015f, -0.015f }, { 0.015f, -0.015f }, { 0.015f, 0.015f }, { -0.015f, 0.015f } };
		const std::vector<V3> wc(4, wire);
		for (int t = 0; t < NTRK; t++) {
			sweepProfileF(v, wf, wc, 0.0f, LEN, 10.0f, [t](float s) { return baseLat(t, s) + 0.1f; }, 5.4f, false, wire);
			sweepProfileF(v, wf, wc, 0.0f, LEN, 10.0f, [t](float s) { return baseLat(t, s) + 0.1f; }, 6.5f, false, wire);
		}
	}

	{
		g_curE = 1.0f;
		for (float s = 75.0f; s < LEN; s += 150.0f) {
			if (inLeadZone(s)) continue;
			bool isNear = false;
			for (const auto& st : g_stations) if (fabsf(relS(s - (st.platformS0 + st.platformS1) * 0.5f)) < 90.0f) isNear = true;
			for (float cs : g_crossS) if (fabsf(relS(s - cs)) < 14.0f) isNear = true;
			if (isNear) continue;
			const TFrame F = trackFrame(s);
			const float lat = fenceEnv(1, s) + 2.2f;
			addBox(v, F.p + F.r * lat + F.u * 3.0f, F.r, F.u, F.f, { 0.07f, 3.2f, 0.07f }, { 0.55f, 0.57f, 0.58f });
			addBox(v, F.p + F.r * (lat - 0.7f) + F.u * 6.3f, F.r, F.u, F.f, { 0.75f, 0.04f, 0.05f }, { 0.55f, 0.57f, 0.58f });
			g_curE = 1.25f;
			addBox(v, F.p + F.r * (lat - 1.4f) + F.u * 6.22f, F.r, F.u, F.f, { 0.35f, 0.05f, 0.14f }, { 0.95f, 0.95f, 0.85f });
			g_curE = 1.0f;
			g_lampPos.push_back(F.p + F.r * (lat - 1.4f) + F.u * 6.05f);
		}
		g_curE = 0.0f;
	}
	return v;
}


static const V3 C_WHITE = { 0.95f, 0.96f, 0.98f }, C_BLUE = { 0.13f, 0.33f, 0.86f };
static const V3 C_SILVER = { 0.80f, 0.82f, 0.85f }, C_ORANGE = { 0.97f, 0.52f, 0.10f };
static const float Y_BLUE1 = 1.52f;
static const V3 C_GLASS = { 0.09f, 0.12f, 0.18f }, C_GLASS2 = { 0.15f, 0.20f, 0.28f };
static const V3 C_DARK = { 0.15f, 0.17f, 0.20f }, C_HOOD = { 0.22f, 0.23f, 0.25f };
static const V3 C_FLOOR = { 0.50f, 0.47f, 0.42f }, C_CEIL = { 0.90f, 0.90f, 0.88f }, C_LINER = { 0.88f, 0.87f, 0.82f };
static const V3 C_SEAT = { 0.20f, 0.42f, 0.55f }, C_SEAT2 = { 0.17f, 0.36f, 0.48f };
static const V3 C_AC = { 0.80f, 0.80f, 0.78f }, C_AC2 = { 0.60f, 0.60f, 0.58f };
static const V3 C_UNDER = { 0.14f, 0.14f, 0.16f }, C_EQ1 = { 0.52f, 0.55f, 0.56f }, C_EQ2 = { 0.46f, 0.49f, 0.50f };
static const V3 C_WHEEL = { 0.20f, 0.20f, 0.22f }, C_AXLE = { 0.12f, 0.12f, 0.14f };
static const V3 C_BOGIE = { 0.16f, 0.17f, 0.20f }, C_MOTOR = { 0.28f, 0.28f, 0.31f };


static void addBoxR(Verts& v, float x0, float x1, float y0, float y1, float z0, float z1, V3 col) {
	addAABB(v, { (x0 + x1) * 0.5f, (y0 + y1) * 0.5f, (z0 + z1) * 0.5f },
		{ (x1 - x0) * 0.5f, (y1 - y0) * 0.5f, (z1 - z0) * 0.5f }, col);
}

static void addSideBox(Verts& v, float sg, float xa, float xb, float y0, float y1, float z0, float z1, V3 col) {
	float a = sg * xa, b = sg * xb;
	addBoxR(v, std::min(a, b), std::max(a, b), y0, y1, z0, z1, col);
}


static void addLivery(Verts& v, float sg, float xa, float xb, float y0, float y1, float z0, float z1) {
	const float yb[4] = { -100.0f, Y_BLUE1, Y_WHITE1, 100.0f };
	const V3 cl[3] = { C_BLUE, C_ORANGE, C_SILVER };
	for (int k = 0; k < 3; k++) {
		float a = std::max(y0, yb[k]), b = std::min(y1, yb[k + 1]);
		if (b - a > 1e-4f) addSideBox(v, sg, xa, xb, a, b, z0, z1, cl[k]);
	}
}
static void addTriV(Verts& v, V3 a, V3 b, V3 c, V3 col, V3 inside) {
	V3 n = norm(cross(b - a, c - a));
	V3 cen = (a + b + c) * (1.0f / 3.0f);
	if (dot(n, cen - inside) < 0) n = n * -1.0f;
	v.push_back({ a, n, col, g_curE }); v.push_back({ b, n, col, g_curE }); v.push_back({ c, n, col, g_curE });
}

static void addHeart(Verts& v, V3 c, V3 r, V3 u, V3 n, float sc) {
	const int N = 36;
	const V3 pal[6] = { { 0.98f, 0.55f, 0.15f }, { 0.98f, 0.85f, 0.20f }, { 0.35f, 0.80f, 0.35f }, { 0.25f, 0.65f, 0.95f }, { 0.55f, 0.35f, 0.85f }, { 0.95f, 0.40f, 0.65f } };
	auto H = [](float t, float& x, float& y) {
		float s = sinf(t);
		x = 16.0f * s * s * s;
		y = 13.0f * cosf(t) - 5.0f * cosf(2.0f * t) - 2.0f * cosf(3.0f * t) - cosf(4.0f * t);
		};
	for (int pass = 0; pass < 2; pass++) {
		const float k = (pass == 0) ? 1.14f : 1.0f, off = (pass == 0) ? 0.004f : 0.010f;
		const V3 base = c + n * off;
		auto P = [&](float x, float y) { return base + r * (x / 32.0f * sc * k) + u * ((y + 2.5f) / 32.0f * sc * k); };
		const V3 ctr = P(0.0f, 0.0f);
		for (int i = 0; i < N; i++) {
			float x0, y0, x1, y1;
			H(6.2831853f * i / N, x0, y0); H(6.2831853f * (i + 1) / N, x1, y1);
			V3 col = (pass == 0) ? V3{ 0.96f, 0.96f, 0.98f } : pal[(i * 6) / N];
			v.push_back({ ctr, n, col, g_curE }); v.push_back({ P(x0, y0), n, col, g_curE }); v.push_back({ P(x1, y1), n, col, g_curE });
		}
	}
}

static void addNumber(Verts& v, V3 c, V3 r, V3 u, V3 n, float dh, const char* s, V3 col) {
	static const unsigned char SEGM[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };
	const float dw = dh * 0.55f, th = dh * 0.13f, pitch = dh * 0.88f, hw = dw * 0.5f, hh = dh * 0.5f;
	const int len = (int)strlen(s);
	for (int i = 0; i < len; i++) {
		int d = s[i] - '0'; if (d < 0 || d > 9) continue;
		V3 ci = c + r * ((i - (len - 1) * 0.5f) * pitch);
		int m = SEGM[d];
		auto seg = [&](float px, float py, float hx, float hy) { addBox(v, ci + r * px + u * py, r, u, n, { hx, hy, 0.003f }, col); };
		if (m & 1)  seg(0.0f, hh - th * 0.5f, hw, th * 0.5f);
		if (m & 2)  seg(hw - th * 0.5f, hh * 0.5f, th * 0.5f, hh * 0.5f);
		if (m & 4)  seg(hw - th * 0.5f, -hh * 0.5f, th * 0.5f, hh * 0.5f);
		if (m & 8)  seg(0.0f, -hh + th * 0.5f, hw, th * 0.5f);
		if (m & 16) seg(-hw + th * 0.5f, -hh * 0.5f, th * 0.5f, hh * 0.5f);
		if (m & 32) seg(-hw + th * 0.5f, hh * 0.5f, th * 0.5f, hh * 0.5f);
		if (m & 64) seg(0.0f, 0.0f, hw, th * 0.5f);
	}
}

struct GR { float x0, y0, x1, y1; };
static const GR GLY_FU[] = {
	{ 1,10.2f,11,11.2f }, { 3.4f,8.4f,4.4f,12 }, { 7.6f,8.4f,8.6f,12 }, { 1,7.2f,11,8.2f }, { 2.4f,5.0f,3.4f,7.4f }, { 8.6f,5.0f,9.6f,7.4f },
	{ 2.4f,5.0f,9.6f,6.0f }, { 3.2f,3.2f,8.8f,4.2f }, { 3.2f,0.2f,8.8f,1.2f }, { 3.2f,1.7f,8.8f,2.7f }, { 3.2f,0.2f,4.2f,4.2f }, { 7.8f,0.2f,8.8f,4.2f } };
static const GR GLY_TSU[] = {
	{ 0.4f,9.6f,1.7f,10.9f }, { 1.0f,2.6f,2.2f,9.2f }, { 1.0f,0.8f,11.6f,2.0f }, { 4.6f,10.6f,11.0f,11.6f }, { 5.0f,3.6f,6.0f,10.6f }, { 9.6f,3.6f,10.6f,10.6f },
	{ 5.0f,7.6f,10.6f,8.6f }, { 5.0f,3.6f,10.6f,4.6f }, { 7.4f,3.6f,8.4f,10.6f } };

static void addBar(Verts& v, V3 a, V3 b, float hx, float hy, V3 col) {
	V3 d = b - a; float L = sqrtf(dot(d, d));
	if (L < 1e-5f) return;
	V3 f = d * (1.0f / L), r = { 1, 0, 0 }, u = cross(f, r);
	addBox(v, (a + b) * 0.5f, r, u, f, { hx, hy, L * 0.5f }, col);
}
static void addPantograph(Verts& v) {
	const V3 cIns = { 0.84f, 0.82f, 0.76f }, cBase = { 0.28f, 0.30f, 0.33f }, cArm = { 0.58f, 0.60f, 0.64f }, cStrip = { 0.10f, 0.10f, 0.11f };
	for (int sx = -1; sx <= 1; sx += 2) for (int k = 0; k < 2; k++) {
		float z = 7.15f + k * 1.0f;
		addBoxR(v, sx * 0.50f - 0.06f, sx * 0.50f + 0.06f, 3.70f, 3.82f, z - 0.06f, z + 0.06f, cIns);
	}
	addBoxR(v, -0.62f, 0.62f, 3.82f, 3.88f, 7.00f, 8.30f, cBase);
	const V3 P0 = { 0.0f, 3.90f, 8.05f }, E = { 0.0f, 4.66f, 7.35f }, C = { 0.0f, 4.78f, 7.85f };
	addBar(v, P0, E, 0.045f, 0.03f, cArm);
	addBar(v, E, C, 0.040f, 0.03f, cArm);
	addBoxR(v, -0.07f, 0.07f, E.y - 0.06f, E.y + 0.06f, E.z - 0.07f, E.z + 0.07f, cBase);
	addBoxR(v, -0.07f, 0.07f, 3.88f, 3.98f, P0.z - 0.07f, P0.z + 0.07f, cBase);
	addBoxR(v, -0.58f, 0.58f, 4.76f, 4.80f, C.z - 0.05f, C.z + 0.05f, cArm);
	addBoxR(v, -0.52f, 0.52f, 4.80f, 4.83f, C.z - 0.045f, C.z + 0.045f, cStrip);
	for (int sx = -1; sx <= 1; sx += 2)
		addBoxR(v, sx * 0.52f - 0.02f, sx * 0.52f + 0.02f, 4.78f, 4.92f, C.z - 0.045f, C.z + 0.045f, cArm);
}

struct Opening { float z0, z1; int kind; };
static void addWindowsIn(std::vector<Opening>& ops, float a, float b, float ma, float mb) {
	const float pillar = 0.25f;
	float inner = (b - a) - ma - mb;
	int n = std::max(1, (int)floorf((inner + pillar) / (1.3f + pillar) + 0.5f));
	float w = (inner - (n - 1) * pillar) / n;
	for (int i = 0; i < n; i++) { float z0 = a + ma + i * (w + pillar); ops.push_back({ z0, z0 + w, 1 }); }
}


static void addWheel(Verts& v, float x, float y, float z) {
	const float r = 0.40f, hw = 0.05f;
	for (int k = 0; k < 4; k++) {
		float a = k * PI / 4.0f, c = cosf(a), s = sinf(a);
		addBox(v, { x, y, z }, { 1,0,0 }, { 0,c,s }, { 0,-s,c }, { hw, r, r * 0.4142f }, C_WHEEL);
	}
}
static void addBogie(Verts& v, float zc) {
	for (int a = -1; a <= 1; a += 2) {
		float z = zc + 1.05f * a;
		for (int sg = -1; sg <= 1; sg += 2) addWheel(v, 0.72f * sg, 0.35f, z);
		addBoxR(v, -0.72f, 0.72f, 0.31f, 0.39f, z - 0.04f, z + 0.04f, C_AXLE);
		addBoxR(v, -0.30f, 0.30f, 0.20f, 0.52f, z - 0.28f, z + 0.28f, C_MOTOR);
	}
	addBoxR(v, 0.84f, 1.00f, 0.42f, 0.60f, zc - 1.45f, zc + 1.45f, C_BOGIE);
	addBoxR(v, -1.00f, -0.84f, 0.42f, 0.60f, zc - 1.45f, zc + 1.45f, C_BOGIE);
	addBoxR(v, -0.90f, 0.90f, 0.44f, 0.58f, zc - 0.12f, zc + 0.12f, C_BOGIE);
}


static void addPrismX(Verts& v, float zs, const float (*pf)[2], int n, float x0, float x1, V3 col) {
	float cy = 0.0f, cz = 0.0f;
	for (int i = 0; i < n; i++) { cz += pf[i][0]; cy += pf[i][1]; }
	cz /= n; cy /= n;
	const V3 inside = { (x0 + x1) * 0.5f, cy, zs * (10.0f + cz) };
	auto P = [&](int i, float x) { return V3{ x, pf[i][1], zs * (10.0f + pf[i][0]) }; };
	for (int i = 0; i < n; i++) {
		int j = (i + 1) % n;
		addTriV(v, P(i, x0), P(i, x1), P(j, x1), col, inside);
		addTriV(v, P(i, x0), P(j, x1), P(j, x0), col, inside);
	}
	for (int e = 0; e < 2; e++) {
		float x = e ? x1 : x0;
		for (int i = 1; i + 1 < n; i++) addTriV(v, P(0, x), P(i, x), P(i + 1, x), col, inside);
	}
}
static const float APRON[5][2] = { { 0.00f, 0.93f }, { 0.24f, 0.74f }, { 0.24f, 0.44f }, { 0.06f, 0.38f }, { 0.00f, 0.38f } };


static void addCabFace(Verts& v, float zs, V3 lamp) {
	auto zb = [&](float x0, float x1, float y0, float y1, float t0, float t1, V3 c) {
		float a = zs * (10.0f + t0), b = zs * (10.0f + t1);
		float xa = zs * x0, xb = zs * x1;
		addBoxR(v, std::min(xa, xb), std::max(xa, xb), y0, y1, std::min(a, b), std::max(a, b), c);
		};

	auto disc = [&](float xc, float yc, float R, float t0, float t1, V3 c) {
		for (int k = 0; k < 4; k++) {
			float a = k * PI / 4.0f, ca = cosf(a), sa = sinf(a);
			addBox(v, { zs * xc, yc, zs * (10.0f + (t0 + t1) * 0.5f) }, { ca, sa, 0 }, { -sa, ca, 0 }, { 0, 0, 1 }, { R, R * 0.4142f, (t1 - t0) * 0.5f }, c);
		}
		};

	auto bar = [&](float x0, float y0, float x1, float y1, float w, float t0, float t1, V3 c) {
		V3 d = { zs * (x1 - x0), y1 - y0, 0 };
		float L = sqrtf(dot(d, d));
		V3 f = d * (1.0f / L), r = { 0, 0, 1 }, u = cross(f, r);
		addBox(v, { zs * (x0 + x1) * 0.5f, (y0 + y1) * 0.5f, zs * (10.0f + (t0 + t1) * 0.5f) }, r, u, f, { (t1 - t0) * 0.5f, w * 0.5f, L * 0.5f }, c);
		};
	const V3 cSil = C_SILVER * 1.04f, cFr = { 0.07f, 0.08f, 0.10f }, cGray = C_SILVER * 0.72f, cMech = { 0.20f, 0.21f, 0.24f };
	g_curE = 0.0f;


	const float GT = 0.06f;
	zb(-1.24f, 1.24f, 3.12f, 3.22f, 0.0f, GT, cFr);
	zb(-1.24f, 1.24f, 1.98f, 2.08f, 0.0f, GT, cFr);
	zb(-1.24f, -1.12f, 2.08f, 3.12f, 0.0f, GT, cFr);
	zb(1.12f, 1.24f, 2.08f, 3.12f, 0.0f, GT, cFr);
	zb(-0.38f, -0.28f, 2.08f, 3.12f, 0.0f, GT, cFr);
	zb(0.48f, 0.58f, 2.08f, 3.12f, 0.0f, GT, cFr);

	zb(-0.28f, 0.48f, 1.12f, 2.15f, 0.0f, 0.04f, cSil);
	zb(-0.28f, 0.48f, 3.00f, 3.12f, 0.0f, 0.04f, cSil);
	zb(-0.28f, -0.20f, 2.15f, 3.00f, 0.0f, 0.04f, cSil);
	zb(0.40f, 0.48f, 2.15f, 3.00f, 0.0f, 0.04f, cSil);
	zb(-0.30f, -0.28f, 1.10f, 3.12f, 0.0f, 0.05f, cFr);
	zb(0.48f, 0.50f, 1.10f, 3.12f, 0.0f, 0.05f, cFr);
	zb(-0.30f, 0.50f, 1.08f, 1.12f, 0.0f, 0.05f, cFr);
	zb(-0.21f, 0.41f, 2.13f, 2.16f, 0.04f, 0.055f, cFr);
	zb(-0.21f, 0.41f, 2.99f, 3.02f, 0.04f, 0.055f, cFr);
	zb(-0.21f, -0.18f, 2.13f, 3.02f, 0.04f, 0.055f, cFr);
	zb(0.38f, 0.41f, 2.13f, 3.02f, 0.04f, 0.055f, cFr);
	zb(0.33f, 0.38f, 1.60f, 1.92f, 0.04f, 0.075f, cFr);
	zb(-0.22f, 0.42f, 1.30f, 1.33f, 0.04f, 0.05f, cGray);
	zb(-0.22f, 0.42f, 1.18f, 1.21f, 0.04f, 0.05f, cGray);


	{
		const V3 inside = { 0.0f, 2.5f, 0.0f };
		for (int sg = -1; sg <= 1; sg += 2) {
			auto fr = [&](float x, float y) { return V3{ zs * sg * x, y, zs * (10.0f + 0.004f) }; };
			const V3 corner = fr(1.35f, 0.90f);
			V3 prev = fr(0.50f, 0.90f);
			for (int i = 1; i <= 10; i++) {
				float t = i / 10.0f, x = 0.50f + 0.85f * t, y = 0.90f + 1.10f * powf(t, 2.3f);
				V3 cur = fr(x, y);
				addTriV(v, corner, prev, cur, C_BLUE, inside);
				prev = cur;
			}
		}
	}


	g_curE = 0.0f;
	zb(-0.17f, 0.37f, 3.22f, 3.47f, 0.0f, 0.05f, cFr);
	disc(-0.03f, 3.345f, 0.085f, 0.050f, 0.062f, C_SILVER * 1.10f);
	disc(0.23f, 3.345f, 0.085f, 0.050f, 0.062f, C_SILVER * 1.10f);
	g_curE = 1.0f;
	disc(-0.03f, 3.345f, 0.062f, 0.062f, 0.072f, lamp);
	disc(0.23f, 3.345f, 0.062f, 0.062f, 0.072f, lamp);

	for (int sg = -1; sg <= 1; sg += 2) {
		g_curE = 0.0f;
		zb(sg * 0.88f, sg * 1.24f, 1.20f, 1.42f, 0.0f, 0.05f, cFr);
		zb(sg * 0.90f, sg * 1.22f, 1.21f, 1.23f, 0.05f, 0.06f, C_SILVER);
		g_curE = 1.0f;
		zb(sg * 0.91f, sg * 1.04f, 1.25f, 1.38f, 0.05f, 0.07f, lamp);
		zb(sg * 1.08f, sg * 1.21f, 1.25f, 1.38f, 0.05f, 0.07f, { 1.0f, 0.15f, 0.10f });
	}

	{
		g_curE = 0.0f; zb(-1.08f, -0.40f, 2.90f, 3.10f, 0.02f, 0.03f, { 0.03f, 0.04f, 0.06f });
		g_curE = 1.0f;
		const V3 wh = { 0.96f, 0.97f, 1.0f };
		auto glyph = [&](const GR* g, int n, float xg, float yg, float gs) {
			const float k = gs / 12.0f;
			for (int i = 0; i < n; i++)
				zb(xg - (g[i].x1 - 6.0f) * k, xg - (g[i].x0 - 6.0f) * k, yg + (g[i].y0 - 6.0f) * k, yg + (g[i].y1 - 6.0f) * k, 0.030f, 0.036f, wh);
			};
		glyph(GLY_FU, (int)(sizeof(GLY_FU) / sizeof(GR)), -0.52f, 3.00f, 0.17f);
		glyph(GLY_TSU, (int)(sizeof(GLY_TSU) / sizeof(GR)), -0.72f, 3.00f, 0.17f);
		zb(-1.04f, -0.90f, 2.93f, 2.96f, 0.030f, 0.036f, wh * 0.8f);
		g_curE = 0.0f;
	}

	addHeart(v, { zs * 0.98f, 1.80f, zs * 10.0f }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, zs }, 0.34f);
	addNumber(v, { zs * -0.82f, 1.80f, zs * 10.012f }, { -zs, 0, 0 }, { 0, 1, 0 }, { 0, 0, zs }, 0.14f, "9210", { 0.08f, 0.08f, 0.10f });


	addPrismX(v, zs, APRON, 5, -1.12f, 1.12f, C_SILVER * 0.70f);
	zb(-1.24f, -1.12f, 0.40f, 0.95f, 0.0f, 0.20f, C_SILVER * 0.60f);
	zb(1.12f, 1.24f, 0.40f, 0.95f, 0.0f, 0.20f, C_SILVER * 0.60f);
	for (int k = 0; k < 3; k++) zb(-1.10f, 1.10f, 0.50f + k * 0.08f, 0.53f + k * 0.08f, 0.235f, 0.255f, cFr);
	zb(-0.34f, 0.34f, 0.50f, 0.82f, 0.24f, 0.36f, cMech);
	zb(-0.13f, 0.13f, 0.58f, 0.74f, 0.36f, 0.62f, { 0.30f, 0.31f, 0.34f });
	zb(-0.19f, 0.19f, 0.54f, 0.78f, 0.62f, 0.76f, { 0.25f, 0.26f, 0.29f });
	zb(-0.08f, 0.08f, 0.60f, 0.72f, 0.76f, 0.79f, { 0.55f, 0.56f, 0.58f });
	for (int sg = -1; sg <= 1; sg += 2) {
		zb(sg * 0.50f, sg * 0.56f, 0.56f, 0.82f, 0.24f, 0.42f, cMech);
		zb(sg * 0.34f, sg * 0.56f, 0.78f, 0.82f, 0.30f, 0.34f, cMech);
	}

	zb(0.04f, 0.16f, 3.70f, 3.96f, -0.80f, -0.68f, cFr);
	zb(0.02f, 0.18f, 3.95f, 4.02f, -0.82f, -0.66f, cFr);
	zb(-0.50f, -0.20f, 3.70f, 3.80f, -1.40f, -0.90f, cGray);
	zb(0.42f, 0.62f, 3.70f, 3.78f, -1.20f, -0.80f, cGray);
}



static float deskTop(float z) { return 1.90f + (z - 9.5f) * 0.1405f + 0.0404f; }
static const float MC_PIVOT[3] = { -0.82f, 2.00f, 9.36f };
static const float DIAL1[3] = { -0.95f, 2.065f, 9.606f };
static const float DIAL2[3] = { -0.70f, 2.065f, 9.606f };

static void addCabInterior(Verts& v, float zs) {
	auto cb = [&](float x0, float x1, float y0, float y1, float z0, float z1, V3 c) {
		float xa = zs * x0, xb = zs * x1, za = zs * z0, zb = zs * z1;
		addBoxR(v, std::min(xa, xb), std::max(xa, xb), y0, y1, std::min(za, zb), std::max(za, zb), c);
		};
	const V3 cGray = { 0.55f, 0.58f, 0.60f }, cDesk = { 0.70f, 0.72f, 0.72f };
	const V3 cBlack = { 0.08f, 0.08f, 0.09f }, cPanel = { 0.13f, 0.14f, 0.16f }, cWhite = { 0.92f, 0.92f, 0.92f };


	cb(-1.25f, 1.25f, Y_FLOOR, 2.08f, 9.87f, 9.90f, C_LINER);
	cb(-1.25f, 1.25f, 3.12f, 3.25f, 9.87f, 9.90f, C_LINER);
	cb(-1.25f, -1.14f, 2.08f, 3.12f, 9.87f, 9.90f, C_LINER);
	cb(1.14f, 1.25f, 2.08f, 3.12f, 9.87f, 9.90f, C_LINER);

	cb(-1.25f, 1.25f, Y_FLOOR, 1.90f, 8.00f, 8.05f, C_LINER);
	cb(-1.25f, 1.25f, 3.10f, 3.25f, 8.00f, 8.05f, C_LINER);
	cb(-1.25f, -1.20f, 1.90f, 3.10f, 8.00f, 8.05f, C_LINER);
	cb(1.20f, 1.25f, 1.90f, 3.10f, 8.00f, 8.05f, C_LINER);
	cb(-0.30f, 0.30f, 1.90f, 3.10f, 8.00f, 8.05f, { 0.80f, 0.80f, 0.76f });
	cb(0.18f, 0.24f, 2.30f, 2.42f, 7.97f, 8.00f, { 0.70f, 0.70f, 0.72f });

	cb(-1.20f, 1.20f, Y_FLOOR, 1.115f, 8.10f, 9.15f, { 0.22f, 0.22f, 0.25f });
	cb(-0.90f, -0.30f, 1.115f, 1.50f, 8.10f, 8.55f, cBlack);
	cb(-0.88f, -0.32f, 1.50f, 2.30f, 8.07f, 8.15f, cBlack);
	cb(-0.86f, -0.34f, 2.10f, 2.30f, 8.06f, 8.07f, { 0.18f, 0.30f, 0.40f });

	cb(-1.25f, 1.25f, Y_FLOOR, 1.90f, 9.15f, 9.87f, cGray);
	{
		const float a = 8.0f * PI / 180.0f, ca = cosf(a), sa = sinf(a);
		addBox(v, { 0, 1.90f, zs * 9.5f }, { zs, 0, 0 }, { 0, ca, -zs * sa }, { 0, sa, zs * ca }, { 1.24f, 0.04f, 0.42f }, cDesk);
	}
	cb(-1.25f, 1.25f, 1.88f, 1.92f, 9.10f, 9.15f, cBlack);

	cb(-1.00f, -0.64f, 1.90f, 2.00f, 9.20f, 9.52f, cPanel);
	cb(-0.835f, -0.805f, 2.00f, 2.004f, 9.25f, 9.47f, cBlack);
	for (int i = 0; i < 15; i++) {
		float z = 9.26f + i * 0.0143f;
		cb(-0.86f, -0.85f, 2.00f, 2.004f, z, z + 0.004f, cWhite);
	}

	cb(-1.13f, -1.05f, 1.93f, 2.00f, 9.30f, 9.42f, cPanel);
	cb(-1.10f, -1.08f, 2.00f, 2.07f, 9.34f, 9.36f, cBlack);
	cb(-1.115f, -1.065f, 2.07f, 2.10f, 9.33f, 9.37f, { 0.80f, 0.20f, 0.20f });

	cb(-0.58f, -0.22f, 1.90f, 1.965f, 9.22f, 9.52f, cPanel);
	for (int c = 0; c < 2; c++) for (int r = 0; r < 2; r++) {
		float x0 = -0.55f + c * 0.20f, z0 = 9.26f + r * 0.12f;
		V3 col = (c == 0) ? V3{ 0.15f, 0.70f, 0.25f } : V3{ 0.90f, 0.80f, 0.20f };
		cb(x0, x0 + 0.10f, 1.965f, 1.99f, z0, z0 + 0.10f, col);
	}

	cb(-0.17f, -0.03f, 1.90f, 1.935f, 9.27f, 9.41f, cBlack);
	cb(-0.14f, -0.06f, 1.935f, 1.99f, 9.30f, 9.38f, { 0.85f, 0.10f, 0.10f });

	cb(0.15f, 0.75f, 1.90f, 1.96f, 9.25f, 9.85f, cPanel);
	for (int i = 0; i < 6; i++) for (int j = 0; j < 3; j++) {
		float x = 0.22f + i * 0.095f, z = 9.38f + j * 0.18f;
		cb(x - 0.018f, x + 0.018f, 1.96f, 2.00f, z - 0.018f, z + 0.018f, cBlack);
		V3 lc = ((i + j) % 3 == 0) ? V3{ 0.85f, 0.25f, 0.20f } : ((i + j) % 3 == 1) ? V3{ 0.80f, 0.80f, 0.82f } : V3{ 0.25f, 0.55f, 0.85f };
		cb(x - 0.008f, x + 0.008f, 2.00f, 2.045f, z - 0.008f, z + 0.008f, lc);
	}
	cb(0.82f, 1.20f, 1.90f, 1.97f, 9.20f, 9.85f, cPanel);
	const V3 lampCols[4] = { { 0.9f, 0.2f, 0.2f }, { 0.95f, 0.8f, 0.2f }, { 0.2f, 0.85f, 0.3f }, { 0.9f, 0.9f, 0.92f } };
	for (int k = 0; k < 4; k++) for (int j = 0; j < 3; j++) {
		float x = 0.88f + k * 0.08f, z = 9.30f + j * 0.20f;
		cb(x - 0.025f, x + 0.025f, 1.97f, 1.99f, z - 0.025f, z + 0.025f, lampCols[k]);
	}

	cb(-0.72f, -0.52f, Y_FLOOR, 1.16f, 9.10f, 9.30f, cBlack);
	cb(-0.45f, -0.35f, Y_FLOOR, 1.15f, 9.15f, 9.28f, cGray);

	cb(-1.15f, -0.22f, 1.897f, 2.16f, 9.62f, 9.88f, cPanel);
	cb(-1.045f, -0.855f, 1.97f, 2.16f, 9.616f, 9.62f, { 0.03f, 0.03f, 0.04f });
	cb(-0.775f, -0.625f, 2.00f, 2.13f, 9.616f, 9.62f, { 0.03f, 0.03f, 0.04f });
	for (int i = 0; i <= 12; i++) {
		float phi = (135.0f - 270.0f * i / 12.0f) * PI / 180.0f, r = 0.072f;
		float px = DIAL1[0] - sinf(phi) * r, py = DIAL1[1] + cosf(phi) * r, h = (i % 2 == 0) ? 0.006f : 0.004f;
		cb(px - h, px + h, py - h, py + h, 9.612f, 9.616f, cWhite);
	}
	for (int i = 0; i <= 8; i++) {
		float phi = (135.0f - 270.0f * i / 8.0f) * PI / 180.0f, r = 0.058f;
		float px = DIAL2[0] - sinf(phi) * r, py = DIAL2[1] + cosf(phi) * r;
		cb(px - 0.004f, px + 0.004f, py - 0.004f, py + 0.004f, 9.612f, 9.616f, cWhite);
	}
	cb(-0.58f, -0.25f, 1.98f, 2.15f, 9.616f, 9.62f, { 0.02f, 0.02f, 0.03f });
	cb(-0.565f, -0.265f, 1.995f, 2.135f, 9.612f, 9.616f, { 0.05f, 0.14f, 0.28f });
	cb(-0.55f, -0.35f, 2.09f, 2.115f, 9.608f, 9.612f, { 0.30f, 0.75f, 0.95f });
	cb(-0.55f, -0.28f, 2.05f, 2.07f, 9.608f, 9.612f, { 0.90f, 0.80f, 0.20f });
	cb(-0.55f, -0.45f, 2.01f, 2.03f, 9.608f, 9.612f, { 0.30f, 0.90f, 0.40f });
	cb(-1.14f, -1.11f, 2.02f, 2.05f, 9.608f, 9.62f, { 1.00f, 0.70f, 0.10f });

	cb(-0.20f, 0.20f, 3.22f, 3.25f, 8.80f, 9.40f, { 1.0f, 1.0f, 0.95f });
}


static Verts buildMcHandle() {
	Verts v;
	addBoxR(v, -0.035f, 0.035f, 0.0f, 0.025f, -0.035f, 0.035f, { 0.75f, 0.77f, 0.80f });
	addBoxR(v, -0.011f, 0.011f, 0.025f, 0.12f, -0.011f, 0.011f, { 0.10f, 0.10f, 0.12f });
	addBoxR(v, -0.09f, 0.09f, 0.12f, 0.15f, -0.018f, 0.018f, { 0.07f, 0.07f, 0.08f });
	addBoxR(v, -0.10f, -0.09f, 0.115f, 0.155f, -0.021f, 0.021f, { 0.60f, 0.60f, 0.65f });
	addBoxR(v, 0.09f, 0.10f, 0.115f, 0.155f, -0.021f, 0.021f, { 0.60f, 0.60f, 0.65f });
	return v;
}
static Verts buildNeedle() {
	Verts v;
	addBoxR(v, -0.004f, 0.004f, -0.012f, 0.07f, -0.002f, 0.002f, { 0.95f, 0.15f, 0.12f });
	addBoxR(v, -0.012f, 0.012f, -0.012f, 0.012f, -0.003f, 0.003f, { 0.85f, 0.85f, 0.88f });
	return v;
}
static Verts buildDoorLamp(V3 c) {
	Verts v;
	addBoxR(v, -1.14f, -1.11f, 2.085f, 2.115f, 9.608f, 9.62f, c);
	return v;
}

static std::vector<Opening> makeOpenings(bool head, bool tail) {
	std::vector<Opening> ops;
	for (int k = 0; k < 3; k++) ops.push_back({ DOOR_Z[k] - DOOR_HW, DOOR_Z[k] + DOOR_HW, 0 });
	addWindowsIn(ops, -5.35f, -0.65f, 0.75f, 0.75f);
	addWindowsIn(ops, 0.65f, 5.35f, 0.75f, 0.75f);
	if (head) { ops.push_back({ 7.40f, 8.00f, 1 }); ops.push_back({ 8.15f, 8.85f, 2 }); }
	else addWindowsIn(ops, 6.65f, 10.0f, 0.75f, 0.35f);
	if (tail) { ops.push_back({ -8.00f, -7.40f, 1 }); ops.push_back({ -8.85f, -8.15f, 2 }); }
	else addWindowsIn(ops, -10.0f, -6.65f, 0.35f, 0.75f);
	std::sort(ops.begin(), ops.end(), [](const Opening& a, const Opening& b) { return a.z0 < b.z0; });
	return ops;
}

static Verts buildCarBody(bool head, bool tail) {
	Verts v;

	addBogie(v, BOGIE_HALF); addBogie(v, -BOGIE_HALF);
	addBoxR(v, -1.00f, 1.00f, 0.80f, 1.00f, -9.0f, 9.0f, C_UNDER);
	addBoxR(v, -1.15f, 1.15f, 0.50f, 0.90f, -5.2f, -2.6f, C_EQ1);
	addBoxR(v, -1.10f, 1.10f, 0.55f, 0.90f, -2.1f, -0.5f, C_EQ2);
	addBoxR(v, -1.15f, 1.15f, 0.50f, 0.90f, 0.2f, 3.4f, C_EQ1);
	addBoxR(v, -1.05f, 1.05f, 0.58f, 0.90f, 3.9f, 5.0f, C_EQ2);


	std::vector<Opening> ops = makeOpenings(head, tail);

	for (int sgi = -1; sgi <= 1; sgi += 2) {
		float sg = (float)sgi;
		auto solid = [&](float a, float b) {
			if (b - a < 1e-4f) return;
			addLivery(v, sg, X_IN, X_OUT, Y_SKIN0, Y_SKIN1, a, b);
			};
		float cur = -10.0f;
		for (const Opening& o : ops) {
			solid(cur, o.z0);
			if (o.kind == 0) {
				addLivery(v, sg, X_IN, X_OUT, Y_SKIN0, Y_FLOOR, o.z0, o.z1);
				addLivery(v, sg, X_IN, X_OUT, Y_WIN1, Y_SKIN1, o.z0, o.z1);
				float zc = (o.z0 + o.z1) * 0.5f;
				addSideBox(v, sg, X_OUT, X_OUT + 0.02f, 3.20f, 3.30f, zc - 0.12f, zc + 0.12f, { 0.95f, 0.55f, 0.15f });
			}
			else if (o.kind == 1) {
				addLivery(v, sg, X_IN, X_OUT, Y_SKIN0, Y_WIN0, o.z0, o.z1);
				addLivery(v, sg, X_IN, X_OUT, Y_WIN1, Y_SKIN1, o.z0, o.z1);
			}
			else {
				addLivery(v, sg, X_IN, X_OUT, Y_SKIN0, Y_FLOOR, o.z0, o.z1);
				addLivery(v, sg, X_IN, X_OUT, 3.20f, Y_SKIN1, o.z0, o.z1);
			}
			cur = o.z1;
		}
		solid(cur, 10.0f);
		{
			const V3 nrm = { sg, 0.0f, 0.0f };
			addHeart(v, { sg * X_OUT, 1.22f, 3.1f }, { 0, 0, 1 }, { 0, 1, 0 }, nrm, 0.38f);
			if (head || tail) addNumber(v, { sg * (X_OUT + 0.012f), 1.22f, 4.5f }, { 0, 0, sg }, { 0, 1, 0 }, nrm, 0.16f, "9210", { 0.97f, 0.97f, 1.0f });
		}

		addSideBox(v, sg, X_OUT, X_OUT + 0.02f, 3.14f, 3.32f, -3.3f, -2.5f, { 0.06f, 0.06f, 0.08f });
		addSideBox(v, sg, X_OUT + 0.02f, X_OUT + 0.025f, 3.20f, 3.26f, -3.2f, -2.6f, { 0.95f, 0.62f, 0.10f });
	}


	const float GANG_HW = 0.65f, GANG_TOP = 3.05f;
	auto endWall = [&](float zs, bool cab) {
		float za = zs > 0 ? 9.9f : -10.0f, zb = zs > 0 ? 10.0f : -9.9f;
		if (!cab) {
			auto band = [&](float x0, float x1, float y0, float y1) {
				if (x1 - x0 < 1e-4f || y1 - y0 < 1e-4f) return;
				if (y0 < Y_BLUE1) addBoxR(v, x0, x1, y0, std::min(y1, Y_BLUE1), za, zb, C_BLUE);
				if (y1 > Y_BLUE1 && y0 < Y_WHITE1) addBoxR(v, x0, x1, std::max(y0, Y_BLUE1), std::min(y1, Y_WHITE1), za, zb, C_ORANGE);
				if (y1 > Y_WHITE1) addBoxR(v, x0, x1, std::max(y0, Y_WHITE1), y1, za, zb, C_SILVER);
				};
			band(-1.35f, -GANG_HW, Y_SKIN0, Y_SKIN1);
			band(GANG_HW, 1.35f, Y_SKIN0, Y_SKIN1);
			band(-GANG_HW, GANG_HW, Y_SKIN0, Y_FLOOR);
			band(-GANG_HW, GANG_HW, GANG_TOP, Y_SKIN1);
			return;
		}
		addBoxR(v, -1.35f, 1.35f, Y_SKIN0, 2.08f, za, zb, C_SILVER);
		addBoxR(v, -1.35f, 1.35f, 3.12f, Y_SKIN1, za, zb, C_SILVER);
		addBoxR(v, -1.35f, -1.14f, 2.08f, 3.12f, za, zb, C_SILVER);
		addBoxR(v, 1.14f, 1.35f, 2.08f, 3.12f, za, zb, C_SILVER);
		};
	endWall(+1.0f, head);
	endWall(-1.0f, tail);
	if (head) addCabFace(v, +1.0f, { 1.0f, 1.0f, 0.85f });
	if (tail) addCabFace(v, -1.0f, { 1.0f, 1.0f, 0.85f });
	g_curE = 2.0f;
	if (head) addCabInterior(v, +1.0f);
	if (tail) addCabInterior(v, -1.0f);
	g_curE = 0.0f;
	{
		const V3 hoodC = { 0.30f, 0.31f, 0.34f };   // 幌(以前より明るい灰色にして見分けやすくする)
		auto hood = [&](float za, float zb) {   // za..zb: 車端から外へ0.6m。隣の車両の幌と0.7m重なり、カーブでも隙間が開かない
			const float zin = (za < 0.0f) ? zb : za;
			const float zf0 = (za < 0.0f) ? za : zin - 0.1f, zf1 = (za < 0.0f) ? zb + 0.1f : zb;   // 床は車内の床の端(±9.9)まで延ばして隙間をなくす
			addBoxR(v, -0.9f, -GANG_HW, 0.95f, 3.20f, za, zb, hoodC);
			addBoxR(v, GANG_HW, 0.9f, 0.95f, 3.20f, za, zb, hoodC);
			addBoxR(v, -GANG_HW, GANG_HW, GANG_TOP, 3.20f, za, zb, hoodC);
			addBoxR(v, -1.05f, 1.05f, 0.92f, Y_FLOOR, zf0, zf1, hoodC);   // 渡り板(幅広・厚め。線路が見える隙間を作らない)
			};
		if (!head) hood(10.0f, 10.6f);
		if (!tail) hood(-10.6f, -10.0f);
	}


	g_curE = 2.0f;
	addBoxR(v, -1.30f, 1.30f, 1.00f, Y_FLOOR, -9.9f, 9.9f, C_FLOOR);
	addBoxR(v, -1.30f, 1.30f, 3.25f, Y_SKIN1, -9.9f, 9.9f, C_CEIL);
	auto linerBox = [&](float sg, float y0, float y1, float za, float zb) {
		float cur = za;
		for (const Opening& o : ops) {
			if (o.kind != 2 || o.z1 <= cur || o.z0 >= zb) continue;
			if (o.z0 > cur + 1e-4f) addSideBox(v, sg, 1.25f, 1.29f, y0, y1, cur, o.z0, C_LINER);
			cur = std::max(cur, o.z1);
		}
		if (zb - cur > 1e-4f) addSideBox(v, sg, 1.25f, 1.29f, y0, y1, cur, zb, C_LINER);
		};
	float segs[4][2] = { { -9.9f, -DOOR_Z[2] - DOOR_HW }, { -5.35f, -0.65f }, { 0.65f, 5.35f }, { DOOR_Z[2] + DOOR_HW, 9.9f } };
	for (int sgi = -1; sgi <= 1; sgi += 2) {
		float sg = (float)sgi;
		for (int i = 0; i < 4; i++) {
			float a = segs[i][0], b = segs[i][1];
			linerBox(sg, Y_FLOOR, Y_WIN0, a, b);
			linerBox(sg, Y_WIN1, 3.25f, a, b);
			float cz = a;
			for (const Opening& o : ops) {
				if (o.kind == 0) continue;
				float oz0 = o.z0 + (o.kind == 2 ? 0.12f : 0.0f), oz1 = o.z1 - (o.kind == 2 ? 0.12f : 0.0f);
				if (oz1 <= a || oz0 >= b) continue;
				if (oz0 - cz > 1e-4f) linerBox(sg, Y_WIN0, Y_WIN1, cz, oz0);
				cz = oz1;
			}
			if (b - cz > 1e-4f) linerBox(sg, Y_WIN0, Y_WIN1, cz, b);
			float a2 = a + 0.15f, b2 = b - 0.15f;
			if (head && i == 3) b2 = 7.9f;
			if (tail && i == 0) a2 = -7.9f;
			addSideBox(v, sg, 0.80f, 1.19f, Y_FLOOR, 1.42f, a2, b2, C_SEAT);
			addSideBox(v, sg, 1.19f, 1.25f, 1.42f, 2.00f, a2, b2, C_SEAT2);
		}
	}

	for (int sgi = -1; sgi <= 1; sgi += 2)
		for (const Opening& o : ops)
			if (o.kind == 0) addSideBox(v, (float)sgi, 1.32f, X_IN, Y_WIN1, 3.25f, o.z0, o.z1, C_LINER);
			else if (o.kind == 2) addSideBox(v, (float)sgi, 1.32f, X_IN, 3.20f, 3.25f, o.z0, o.z1, C_LINER);
	auto linerEnd = [&](float za, float zb) {
		addBoxR(v, -1.30f, -GANG_HW, Y_FLOOR, 3.25f, za, zb, C_LINER);
		addBoxR(v, GANG_HW, 1.30f, Y_FLOOR, 3.25f, za, zb, C_LINER);
		addBoxR(v, -GANG_HW, GANG_HW, GANG_TOP, 3.25f, za, zb, C_LINER);
		};
	if (!head) linerEnd(9.86f, 9.9f);
	if (!tail) linerEnd(-9.9f, -9.86f);
	g_curE = 0.0f;

	addBoxR(v, -1.40f, 1.40f, Y_SKIN1, 3.50f, -10.0f, 10.0f, C_SILVER);
	addBoxR(v, -1.05f, 1.05f, 3.50f, 3.64f, -10.0f, 10.0f, { 0.82f, 0.84f, 0.86f });
	addBoxR(v, -0.70f, 0.70f, 3.64f, 3.70f, -10.0f, 10.0f, { 0.70f, 0.72f, 0.74f });
	const float acZ[2] = { -4.2f, 3.6f };
	for (int i = 0; i < 2; i++) {
		addBoxR(v, -0.68f, 0.68f, 3.70f, 3.88f, acZ[i] - 1.4f, acZ[i] + 1.4f, C_AC);
		addBoxR(v, -0.50f, 0.50f, 3.88f, 3.93f, acZ[i] - 1.1f, acZ[i] + 1.1f, C_AC2);
	}
	if (!head && !tail) addPantograph(v);
	return v;
}


static const V3 C_PANE = { 0.62f, 0.84f, 0.96f };
static const V3 C_WSC = { 0.12f, 0.19f, 0.25f };
static void addGlassQuadX(Verts& v, float x, float y0, float y1, float z0, float z1, V3 col, float nx) {
	V3 n = { nx, 0, 0 };
	V3 q[4] = { { x, y0, z0 }, { x, y1, z0 }, { x, y1, z1 }, { x, y0, z1 } };
	int idx[6] = { 0, 1, 2, 0, 2, 3 };
	for (int i : idx) v.push_back({ q[i], n, col });
}
static void addGlassQuadZ(Verts& v, float z, float x0, float x1, float y0, float y1, V3 col, float nz) {
	V3 n = { 0, 0, nz };
	V3 q[4] = { { x0, y0, z }, { x1, y0, z }, { x1, y1, z }, { x0, y1, z } };
	int idx[6] = { 0, 1, 2, 0, 2, 3 };
	for (int i : idx) v.push_back({ q[i], n, col });
}

static Verts buildWindscreen(float zs) {
	Verts v;
	float z = zs * 10.012f;
	addGlassQuadZ(v, z, -1.14f, 1.14f, 2.08f, 3.12f, C_WSC, zs);
	return v;
}

static void addCabGlass(Verts& v, float zs) {
	float z = zs * 8.025f;
	float xa = zs * -1.20f, xb = zs * -0.30f, xc = zs * 0.30f, xd = zs * 1.20f;
	addGlassQuadZ(v, z, std::min(xa, xb), std::max(xa, xb), 1.90f, 3.10f, C_PANE, zs);
	addGlassQuadZ(v, z, std::min(xc, xd), std::max(xc, xd), 1.90f, 3.10f, C_PANE, zs);
}
static Verts buildCarGlass(bool head, bool tail) {
	Verts v;
	std::vector<Opening> ops = makeOpenings(head, tail);
	for (int sgi = -1; sgi <= 1; sgi += 2) {
		float sg = (float)sgi;
		for (const Opening& o : ops)
			if (o.kind == 1) addGlassQuadX(v, sg * 1.36f, Y_WIN0, Y_WIN1, o.z0, o.z1, C_PANE, sg);
	}
	if (head) addCabGlass(v, +1.0f);
	if (tail) addCabGlass(v, -1.0f);
	return v;
}
static Verts buildDoorGlass(float sg) {
	Verts v;
	addGlassQuadX(v, sg * 1.365f, 2.05f, 2.90f, -0.14f, 0.14f, C_PANE, sg);
	return v;
}


static Verts buildDoorLeaf(float sg) {
	Verts v;
	const float h = LEAF_HW;
	addLivery(v, sg, X_IN, X_OUT, Y_FLOOR, 2.05f, -h, h);
	addLivery(v, sg, X_IN, X_OUT, 2.90f, Y_WIN1, -h, h);
	addLivery(v, sg, X_IN, X_OUT, 2.05f, 2.90f, -h, -0.14f);
	addLivery(v, sg, X_IN, X_OUT, 2.05f, 2.90f, 0.14f, h);
	const float e0 = g_curE; g_curE = 2.0f;
	addSideBox(v, sg, 1.32f, X_IN, Y_FLOOR, 2.05f, -h, h, C_LINER);
	addSideBox(v, sg, 1.32f, X_IN, 2.90f, Y_WIN1, -h, h, C_LINER);
	addSideBox(v, sg, 1.32f, X_IN, 2.05f, 2.90f, -h, -0.14f, C_LINER);
	addSideBox(v, sg, 1.32f, X_IN, 2.05f, 2.90f, 0.14f, h, C_LINER);
	g_curE = e0;
	return v;
}



static const float CAB_DOOR_Z0 = 8.15f, CAB_DOOR_Z1 = 8.85f, CAB_HINGE_X = 1.375f;
static Verts buildCabDoor(float sg, float zs) {
	Verts v;
	const float za = zs > 0 ? CAB_DOOR_Z0 : -CAB_DOOR_Z1, zb = zs > 0 ? CAB_DOOR_Z1 : -CAB_DOOR_Z0;
	addLivery(v, sg, X_IN, X_OUT, Y_FLOOR, 2.15f, za, zb);
	addLivery(v, sg, X_IN, X_OUT, 2.15f, 3.00f, za, za + 0.12f);
	addLivery(v, sg, X_IN, X_OUT, 2.15f, 3.00f, zb - 0.12f, zb);
	addLivery(v, sg, X_IN, X_OUT, 3.00f, 3.20f, za, zb);
	addSideBox(v, sg, X_OUT, X_OUT + 0.02f, Y_FLOOR, 1.13f, za, zb, C_DARK);
	addSideBox(v, sg, X_OUT, X_OUT + 0.02f, 1.13f, 3.17f, za, za + 0.03f, C_DARK);
	addSideBox(v, sg, X_OUT, X_OUT + 0.02f, 1.13f, 3.17f, zb - 0.03f, zb, C_DARK);
	addSideBox(v, sg, X_OUT, X_OUT + 0.02f, 3.17f, 3.20f, za, zb, C_DARK);
	const float hz = zs > 0 ? za + 0.06f : zb - 0.12f;
	addSideBox(v, sg, X_OUT, X_OUT + 0.03f, 2.00f, 2.10f, hz, hz + 0.06f, { 0.7f, 0.7f, 0.72f });
	const float e0 = g_curE; g_curE = 2.0f;
	addSideBox(v, sg, 1.32f, X_IN, Y_FLOOR, 2.15f, za, zb, C_LINER);
	addSideBox(v, sg, 1.32f, X_IN, 2.15f, 3.00f, za, za + 0.12f, C_LINER);
	addSideBox(v, sg, 1.32f, X_IN, 2.15f, 3.00f, zb - 0.12f, zb, C_LINER);
	addSideBox(v, sg, 1.32f, X_IN, 3.00f, 3.20f, za, zb, C_LINER);
	g_curE = e0;
	return v;
}
static Verts buildCabDoorGlass(float sg, float zs) {
	Verts v;
	const float za = zs > 0 ? CAB_DOOR_Z0 : -CAB_DOOR_Z1, zb = zs > 0 ? CAB_DOOR_Z1 : -CAB_DOOR_Z0;
	addGlassQuadX(v, sg * CAB_HINGE_X, 2.15f, 3.00f, za + 0.12f, zb - 0.12f, C_PANE, sg);
	return v;
}

static M4 cabDoorMat(float sg, float zs, float pos) {
	float t = std::min(1.0f, std::max(0.0f, pos)); t = t * t * (3.0f - 2.0f * t);
	const float th = sg * zs * t * (PI * 0.5f);
	const float hx = sg * CAB_HINGE_X, hz = zs * CAB_DOOR_Z1;
	M4 a = basisMat({ 1,0,0 }, { 0,1,0 }, { 0,0,1 }, { -hx, 0, -hz });
	M4 r = basisMat({ cosf(th), 0, -sinf(th) }, { 0,1,0 }, { sinf(th), 0, cosf(th) }, { 0,0,0 });
	M4 c = basisMat({ 1,0,0 }, { 0,1,0 }, { 0,0,1 }, { hx, 0, hz });
	return mul(mul(a, r), c);
}




static float sm01(float t) { t = std::min(1.0f, std::max(0.0f, t)); return t * t * (3.0f - 2.0f * t); }

struct Bump { float x, z, h, r; };
static std::vector<Bump> g_bumps;
static float g_cx = 0, g_cz = 0, g_R = 600.0f, g_phi0 = 0, g_phiDir = 1;
static float g_nOff[4] = { 0, 0, 0, 0 }, g_rvPh[4] = { 0, 0, 0, 0 };
static const float WATER_Y = -6.0f, LAKE_R = 170.0f;
static const float TERR_CELL = 16.0f;
static const int   TERR_N = 306;

static float hashf(int x, int z) {
	unsigned h = (unsigned)x * 374761393u + (unsigned)z * 668265263u;
	h = (h ^ (h >> 13)) * 1274126177u; h ^= h >> 16;
	return (h & 0xFFFFFFu) / 16777215.0f;
}
static float vnoise(float x, float z) {
	int xi = (int)floorf(x), zi = (int)floorf(z);
	float fx = sm01(x - xi), fz = sm01(z - zi);
	float a = hashf(xi, zi), b = hashf(xi + 1, zi), c = hashf(xi, zi + 1), d = hashf(xi + 1, zi + 1);
	float t0 = a + (b - a) * fx, t1 = c + (d - c) * fx;
	return t0 + (t1 - t0) * fz;
}

static void sceneryOrigin() {
	V3 c = { 0, 0, 0 }; const int n = 72;
	for (int i = 0; i < n; i++) c = c + trackAt(g_totalTrackLen * i / n).p;
	g_cx = c.x / n; g_cz = c.z / n;
	g_R = g_segments[0].radius;
	V3 p0 = trackAt(0).p, p1 = trackAt(g_R * 0.1f).p;
	g_phi0 = atan2f(p0.z - g_cz, p0.x - g_cx);
	float d = atan2f(p1.z - g_cz, p1.x - g_cx) - g_phi0;
	while (d > PI) d -= 2 * PI;
	while (d < -PI) d += 2 * PI;
	g_phiDir = (d >= 0) ? 1.0f : -1.0f;
}

static void polar(float x, float z, float& r, float& s) {
	float dx = x - g_cx, dz = z - g_cz;
	r = sqrtf(dx * dx + dz * dz);
	float d = (atan2f(dz, dx) - g_phi0) * g_phiDir;
	d = fmodf(d, 2 * PI); if (d < 0) d += 2 * PI;
	s = d * g_R;
}
static float riverR(float phi) { return 1350.0f + 130.0f * sinf(3 * phi + g_rvPh[0]) + 70.0f * sinf(7 * phi + g_rvPh[1]) + 40.0f * sinf(11 * phi + g_rvPh[2]); }
static float riverW(float phi) { return 26.0f + 6.0f * sinf(5 * phi + g_rvPh[3]); }
static float forestMask(float x, float z) { return sm01((vnoise(x / 200.0f + g_nOff[2], z / 200.0f + g_nOff[3]) - 0.45f) / 0.2f); }


static const int NHD = 4;
struct HouseState {
	bool valid = false;
	V3 O = { 0, 0, 0 };
	V3 R = { 1, 0, 0 }, F = { 0, 0, 1 };
	float padY = 0.0f;
	M4 M = {};
	float doorPos[NHD] = { 0, 0, 0, 0 };
	bool  doorTgt[NHD] = { false, false, false, false };
	bool  tapOn = false; float tapLvl = 0.0f;
	bool  pcOn = false;  float pcLvl = 0.0f;
	bool  roomOn[2] = { false, true };
	float roomLvl[2] = { 0, 1 };
	bool  lampOn = false;  float lampLvl = 0.0f;
	bool  stoveOn = false; float stoveLvl = 0.0f;
	float steam = 0.0f;
};
static HouseState g_hs;
static V3 hzW(V3 l) { return g_hs.O + g_hs.R * l.x + V3{ 0, 1, 0 } * l.y + g_hs.F * l.z; }
static V3 hzL(V3 w) { V3 d = w - g_hs.O; return { dot(d, g_hs.R), d.y, dot(d, g_hs.F) }; }
static V3 hzWDir(V3 d) { return g_hs.R * d.x + V3{ 0, 1, 0 } * d.y + g_hs.F * d.z; }
static V3 hzLDir(V3 d) { return { dot(d, g_hs.R), d.y, dot(d, g_hs.F) }; }

static float terrainH(float x, float z) {
	float r, s; polar(x, z, r, s);
	float adr = fabsf(r - g_R);
	float A = 1.0f - sm01((adr - 60.0f) / 290.0f);
	float M = sm01((adr - 40.0f) / 260.0f);
	float f = 0.0f;
	float rn = 0.8f + 0.4f * vnoise(x * 0.01f + g_nOff[0], z * 0.01f + g_nOff[1]);
	for (const Bump& b : g_bumps) {
		float dx = x - b.x, dz = z - b.z, d2 = dx * dx + dz * dz;
		if (d2 >= b.r * b.r) continue;
		float t = 1.0f - sqrtf(d2) / b.r;
		f += (b.h > 0) ? b.h * powf(t, 1.7f) * rn : b.h * sm01(t);
	}
	float rough = 6.0f * vnoise(x / 260.0f + g_nOff[1], z / 260.0f + g_nOff[0]) + 3.0f * vnoise(x / 110.0f, z / 110.0f) + 1.2f * vnoise(x / 45.0f, z / 45.0f) - 5.1f;
	float natural = (f + rough) * M;
	float h = A * (trackHeight(s) - 0.2f) + (1.0f - A) * natural;
	if (g_hs.valid) {
		const float dx = x - g_hs.O.x, dz = z - g_hs.O.z;
		const float w = (1.0f - sm01((sqrtf(dx * dx + dz * dz) - 14.0f) / 26.0f)) * sm01((adr - 8.0f) / 10.0f);
		h += (g_hs.padY - h) * w;
	}
	const float bed = WATER_Y - 3.0f;
	float phi = atan2f(z - g_cz, x - g_cx);
	float cw = sm01((fabsf(r - riverR(phi)) - (riverW(phi) - 6.0f)) / 45.0f);
	h = std::min(h, bed + (std::max(h, bed) - bed) * cw);
	float cl = sm01((r - (LAKE_R - 20.0f)) / 60.0f);
	h = std::min(h, bed + (std::max(h, bed) - bed) * cl);
	float fade = 1.0f - sm01((r - 2150.0f) / 300.0f);
	return -59.3f + (h + 59.3f) * fade;
}

static void addTri(Verts& v, V3 a, V3 b, V3 c, V3 col, V3 outward) {
	V3 n = norm(cross(b - a, c - a));
	V3 cen = (a + b + c) * (1.0f / 3.0f);
	if (dot(n, cen - outward) < 0) n = n * -1.0f;
	v.push_back({ a, n, col, g_curE }); v.push_back({ b, n, col, g_curE }); v.push_back({ c, n, col, g_curE });
}
static void addPyramid(Verts& v, V3 base, float yaw, float hw, float hd, float h, V3 col) {
	float cy = cosf(yaw), sy = sinf(yaw);
	V3 rr = { cy, 0, -sy }, ff = { sy, 0, cy };
	V3 q[4] = { base - rr * hw - ff * hd, base + rr * hw - ff * hd, base + rr * hw + ff * hd, base - rr * hw + ff * hd };
	V3 apex = base + V3{ 0, h, 0 }, inner = base + V3{ 0, h * 0.3f, 0 };
	for (int i = 0; i < 4; i++) addTri(v, q[i], q[(i + 1) % 4], apex, col, inner);
}

static void addTrunk(Verts& v, V3 p, float h, float r0, float r1, V3 col) {
	const int S = 8;
	for (int i = 0; i < S; i++) {
		float a0 = 6.2831853f * i / S, a1 = 6.2831853f * (i + 1) / S;
		V3 b0 = p + V3{ cosf(a0) * r0, 0, sinf(a0) * r0 }, b1 = p + V3{ cosf(a1) * r0, 0, sinf(a1) * r0 };
		V3 t0 = p + V3{ cosf(a0) * r1, h, sinf(a0) * r1 }, t1 = p + V3{ cosf(a1) * r1, h, sinf(a1) * r1 };
		V3 n0 = { cosf(a0), 0.1f, sinf(a0) }, n1 = { cosf(a1), 0.1f, sinf(a1) };
		V3 cb = col * 0.75f;
		pushV(v, b0, norm(n0), cb); pushV(v, b1, norm(n1), cb); pushV(v, t1, norm(n1), col);
		pushV(v, b0, norm(n0), cb); pushV(v, t1, norm(n1), col); pushV(v, t0, norm(n0), col);
	}
}
static void addConeTier(Verts& v, V3 base, float rot, float R, float H, V3 col) {
	const int S = 12;
	V3 apex = base + V3{ 0, H, 0 };
	const V3 cb = col * 0.78f, ct = col * 1.12f;
	for (int i = 0; i < S; i++) {
		float a0 = rot + 6.2831853f * i / S, a1 = rot + 6.2831853f * (i + 1) / S, am = (a0 + a1) * 0.5f;
		float r0 = R * (1.0f + 0.10f * sinf(a0 * 3.0f + rot * 5.0f)), r1 = R * (1.0f + 0.10f * sinf(a1 * 3.0f + rot * 5.0f));
		V3 p0 = base + V3{ cosf(a0) * r0, 0, sinf(a0) * r0 }, p1 = base + V3{ cosf(a1) * r1, 0, sinf(a1) * r1 };
		pushV(v, p0, norm(V3{ cosf(a0) * H, R, sinf(a0) * H }), cb);
		pushV(v, p1, norm(V3{ cosf(a1) * H, R, sinf(a1) * H }), cb);
		pushV(v, apex, norm(V3{ cosf(am) * H, R, sinf(am) * H }), ct);
	}
}
static void addBlob(Verts& v, V3 c, V3 rad, float seed, V3 col) {
	const int SL = 8, ST = 4;
	auto P = [&](int i, int j, V3& nrm, V3& cl) {
		float th = 6.2831853f * i / SL, ph = 3.14159265f * j / ST;
		float sx = sinf(ph) * cosf(th), sy = cosf(ph), sz = sinf(ph) * sinf(th);
		float k = 1.0f + 0.16f * sinf(th * 2.0f + seed) * sinf(ph * 2.0f + seed * 0.7f) + 0.08f * sinf(th * 5.0f + seed * 1.3f);
		nrm = norm(V3{ sx / rad.x, sy / rad.y, sz / rad.z });
		cl = col * (0.72f + 0.40f * (sy * 0.5f + 0.5f));
		return c + V3{ sx * rad.x * k, sy * rad.y * k, sz * rad.z * k };
		};
	for (int i = 0; i < SL; i++) for (int j = 0; j < ST; j++) {
		V3 na, nb, nc, nd, ca, cb2, cc, cd;
		V3 pa = P(i, j, na, ca), pb = P(i + 1, j, nb, cb2), pc = P(i + 1, j + 1, nc, cc), pd = P(i, j + 1, nd, cd);
		if (dot(cross(pb - pa, pc - pa), cross(pb - pa, pc - pa)) > 1e-10f) { pushV(v, pa, na, ca); pushV(v, pb, nb, cb2); pushV(v, pc, nc, cc); }
		if (dot(cross(pc - pa, pd - pa), cross(pc - pa, pd - pa)) > 1e-10f) { pushV(v, pa, na, ca); pushV(v, pc, nc, cc); pushV(v, pd, nd, cd); }
	}
}
static void addConifer(Verts& v, V3 p, float yaw, float sc, V3 leaf) {
	addTrunk(v, p, 2.4f * sc, 0.26f * sc, 0.13f * sc, { 0.30f, 0.20f, 0.12f });
	const int T = 5;
	for (int t = 0; t < T; t++) {
		float f = (float)t / (T - 1);
		addConeTier(v, p + V3{ 0, (1.5f + 3.7f * f) * sc, 0 }, yaw + t * 0.7f, (2.4f - 1.5f * f) * sc, (3.4f - 0.8f * f) * sc, leaf * (0.92f + 0.22f * f));
	}
}
static void addBroadleaf(Verts& v, V3 p, float yaw, float sc, V3 leaf) {
	float cy = cosf(yaw), sy = sinf(yaw);
	V3 rr = { cy, 0, -sy }, ff = { sy, 0, cy };
	addTrunk(v, p, 3.4f * sc, 0.30f * sc, 0.16f * sc, { 0.34f, 0.23f, 0.14f });
	addBlob(v, p + V3{ 0, 4.4f * sc, 0 }, { 2.2f * sc, 1.9f * sc, 2.2f * sc }, yaw, leaf);
	addBlob(v, p + rr * (1.2f * sc) + ff * (0.6f * sc) + V3{ 0, 3.8f * sc, 0 }, { 1.4f * sc, 1.2f * sc, 1.4f * sc }, yaw + 1.7f, leaf * 0.95f);
	addBlob(v, p + rr * (-1.0f * sc) + ff * (-0.5f * sc) + V3{ 0, 4.9f * sc, 0 }, { 1.4f * sc, 1.2f * sc, 1.4f * sc }, yaw + 3.1f, leaf * 1.06f);
	addBlob(v, p + V3{ 0, 5.7f * sc, 0 }, { 1.3f * sc, 1.0f * sc, 1.3f * sc }, yaw + 4.4f, leaf * 1.12f);
}
static void addHouse(Verts& v, V3 p, float yaw, float w, float d, float h, V3 wall, V3 roof, float drop) {
	float cy = cosf(yaw), sy = sinf(yaw);
	V3 rr = { cy, 0, -sy }, ff = { sy, 0, cy }, uu = { 0, 1, 0 };
	const float ca = cosf(0.55f), sa = sinf(0.55f), ta = sa / ca;
	float fh = drop + 1.0f + 0.15f;
	addBox(v, p + uu * (0.15f - fh * 0.5f), rr, uu, ff, { w * 0.5f + 0.1f, fh * 0.5f, d * 0.5f + 0.1f }, { 0.52f, 0.52f, 0.50f });
	addBox(v, p + uu * (0.15f + h * 0.5f), rr, uu, ff, { w * 0.5f, h * 0.5f, d * 0.5f }, wall);
	V3 ridge = p + uu * (0.15f + h + (w * 0.5f) * ta + 0.07f);
	float Ls = (w * 0.5f) / ca + 0.5f;
	V3 rl = rr * ca + uu * sa, ul = rr * (-sa) + uu * ca;
	addBox(v, ridge + rr * (-(Ls * 0.5f) * ca) + uu * (-(Ls * 0.5f) * sa), rl, ul, ff, { Ls * 0.5f, 0.07f, d * 0.5f + 0.4f }, roof);
	V3 rr2 = rr * ca - uu * sa, ur = rr * sa + uu * ca;
	addBox(v, ridge + rr * ((Ls * 0.5f) * ca) + uu * (-(Ls * 0.5f) * sa), rr2, ur, ff, { Ls * 0.5f, 0.07f, d * 0.5f + 0.4f }, roof);
	for (int sg = -1; sg <= 1; sg += 2) {
		V3 base = p + ff * (sg * d * 0.5f) + uu * (0.15f + h);
		addTri(v, base - rr * (w * 0.5f), base + rr * (w * 0.5f), base + uu * ((w * 0.5f) * ta), wall, p + uu * (0.15f + h * 0.5f));
	}
	const V3 win = { 0.30f, 0.42f, 0.58f };
	addBox(v, p + ff * (d * 0.5f + 0.02f) + rr * (w * 0.22f) + uu * (0.15f + 0.95f), rr, uu, ff, { 0.45f, 0.95f, 0.03f }, { 0.38f, 0.24f, 0.14f });
	addBox(v, p + ff * (d * 0.5f + 0.02f) + rr * (-w * 0.22f) + uu * (0.15f + 1.6f), rr, uu, ff, { 0.45f, 0.40f, 0.03f }, win);
	for (int sg = -1; sg <= 1; sg += 2)
		addBox(v, p + rr * (sg * (w * 0.5f + 0.02f)) + uu * (0.15f + 1.6f), rr, uu, ff, { 0.03f, 0.40f, 0.50f }, win);
}




struct HDoorDef { V3 hinge, closedDir, openDir; float w, h, maxAng; };
static const HDoorDef H_DOORS[NHD] = {
	{ { 3.70f, 0.00f, 3.90f }, { -1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, -1.0f }, 0.96f, 2.05f, 1.75f },
	{ { 3.05f, 2.75f, -3.55f }, { 0.0f, 0.0f, 1.0f }, { -1.0f, 0.0f, 0.0f }, 0.86f, 2.05f, 1.75f },
	{ { -4.25f, 0.02f, -3.13f }, { 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, 0.65f, 1.18f, 1.55f },
	{ { -4.25f, 1.22f, -3.13f }, { 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, 0.65f, 0.58f, 1.55f },
};
struct HLightDef { V3 pos; float range; V3 col; float down; V3 mn, mx; };
static const HLightDef H_LIGHTS[4] = {
	{ { 0.20f, 2.30f, 0.20f }, 11.0f, { 1.55f, 1.47f, 1.30f }, 0.0f, { -4.34f, -0.05f, -3.84f }, { 4.34f, 2.66f, 3.84f } },
	{ { -0.60f, 4.95f, 0.20f }, 11.0f, { 1.55f, 1.47f, 1.30f }, 0.0f, { -4.34f, 2.71f, -3.84f }, { 4.34f, 5.19f, 3.84f } },
	{ { -2.30f, 3.62f, 3.30f }, 2.8f, { 1.80f, 1.40f, 0.90f }, 0.9f, { -4.34f, 2.71f, -3.84f }, { 4.34f, 5.19f, 3.84f } },
	{ { -1.85f, 1.18f, -3.45f }, 2.4f, { 0.55f, 0.36f, 0.20f }, 0.0f, { -4.34f, -0.05f, -3.84f }, { 4.34f, 2.54f, 3.84f } },
};
struct HChair { float x, z, s, yb, sx, sz; };
static const int H_NCHAIR = 5;
static const HChair H_CHAIRS[H_NCHAIR] = {
	{ 0.0f, -0.75f, 1.0f, 0.0f, -0.65f, -0.75f },
	{ 0.8f, -0.75f, 1.0f, 0.0f, 1.45f, -0.75f },
	{ 0.0f, 1.25f, -1.0f, 0.0f, -0.65f, 1.25f },
	{ 0.8f, 1.25f, -1.0f, 0.0f, 1.45f, 1.25f },
	{ -2.6f, 2.50f, 1.0f, 2.75f, -2.6f, 1.55f },
};
struct HTarget { int kind, idx; V3 mn, mx; };
static const HTarget H_TGT[6] = {
	{ 11, 0, { 2.05f, 1.00f, 3.45f }, { 2.45f, 1.40f, 3.84f } },
	{ 11, 1, { 2.70f, 3.70f, -2.65f }, { 3.04f, 4.20f, -2.15f } },
	{ 12, 0, { -2.14f, 0.00f, -3.84f }, { -1.16f, 1.16f, -3.08f } },
	{ 13, 0, { -2.46f, 3.46f, 3.14f }, { -1.90f, 3.86f, 3.46f } },
	{ 14, 0, { -2.90f, 0.88f, -3.80f }, { -2.20f, 1.25f, -3.25f } },
	{ 15, 0, { -2.92f, 3.46f, 3.05f }, { -2.46f, 3.76f, 3.46f } },
};
static const V3 H_BED_SIT = { -3.675f, 3.34f, -3.00f }, H_BED_STAND = { -2.65f, 2.77f, -2.60f };


static const float PC_X0 = -2.83f, PC_X1 = -2.52f, PC_Y0 = 3.505f, PC_Y1 = 3.715f, PC_ZB = 3.378f;
struct HouseVerts {
	Verts stat;
	Verts glass;
	Verts door[NHD];
	Verts glow[3];
	Verts flame;
	Verts sphere;
	Verts water, pcStatic, pcBar[6], pcNeedle, pcFill;
};
static HouseVerts g_hvx;


static void houseInit() {
	const float sH = g_stations[0].stopS - 14.0f;
	const TFrame T = trackFrame(sH);
	const V3 Fz = norm({ T.r.x, 0.0f, T.r.z });
	const V3 out = Fz * -1.0f;
	V3 C = T.p + out * 28.0f;
	const float cell = TERR_CELL, X0 = g_cx - TERR_N * cell * 0.5f, Z0 = g_cz - TERR_N * cell * 0.5f;
	for (int it = 0; it < 4; it++) {
		C.x = X0 + (floorf((C.x - X0) / cell) + 0.5f) * cell;
		C.z = Z0 + (floorf((C.z - Z0) / cell) + 0.5f) * cell;
		const float lat = dot(C - T.p, out);
		if (lat >= 22.0f) break;
		C = C + out * (22.0f - lat + 8.0f);
	}
	g_hs.valid = false;
	const float h0 = terrainH(C.x, C.z);
	g_hs.padY = h0;
	g_hs.O = { C.x, h0 + 0.15f, C.z };
	g_hs.F = Fz; g_hs.R = rightOf(Fz);
	g_hs.M = basisMat(g_hs.R, { 0, 1, 0 }, g_hs.F, g_hs.O);
	g_hs.valid = true;
}

struct WOpen { float a0, a1, y0, y1; };
static void hzPieces(Verts& v, int axis, float c0, float c1, float a0, float a1, float y0, float y1, const std::vector<WOpen>& ops, V3 col) {
	auto box = [&](float p0, float p1, float q0, float q1) {
		if (p1 - p0 < 1e-4f || q1 - q0 < 1e-4f) return;
		if (axis == 0) addBoxR(v, p0, p1, q0, q1, c0, c1, col);
		else addBoxR(v, c0, c1, q0, q1, p0, p1, col);
		};
	float cur = a0;
	for (const WOpen& o : ops) {
		box(cur, o.a0, y0, y1);
		box(o.a0, o.a1, y0, o.y0);
		box(o.a0, o.a1, o.y1, y1);
		cur = o.a1;
	}
	box(cur, a1, y0, y1);
}

static void hzWall(Verts& v, int axis, float c0, float c1, int extSide, float a0, float a1, float y0, float y1, std::vector<WOpen> ops, V3 colExt, V3 colInt) {
	std::sort(ops.begin(), ops.end(), [](const WOpen& p, const WOpen& q) { return p.a0 < q.a0; });
	if (extSide == 0) { hzPieces(v, axis, c0, c1, a0, a1, y0, y1, ops, colInt); return; }
	const float te = 0.12f;
	if (extSide > 0) { hzPieces(v, axis, c1 - te, c1, a0, a1, y0, y1, ops, colExt); hzPieces(v, axis, c0, c1 - te, a0, a1, y0, y1, ops, colInt); }
	else { hzPieces(v, axis, c0, c0 + te, a0, a1, y0, y1, ops, colExt); hzPieces(v, axis, c0 + te, c1, a0, a1, y0, y1, ops, colInt); }
}

static void hzWindow(Verts& v, Verts& g, int axis, float c0, float c1, int extSide, float a0, float a1, float y0, float y1) {
	const V3 fc = { 0.97f, 0.97f, 0.95f }, cPane = { 0.62f, 0.84f, 0.96f };
	const float b = 0.05f;
	auto box = [&](float p0, float p1, float q0, float q1, float r0, float r1) {
		if (axis == 0) addBoxR(v, p0, p1, q0, q1, r0, r1, fc); else addBoxR(v, r0, r1, q0, q1, p0, p1, fc);
		};
	box(a0, a0 + b, y0, y1, c0, c1);
	box(a1 - b, a1, y0, y1, c0, c1);
	box(a0 + b, a1 - b, y0, y0 + b, c0, c1);
	box(a0 + b, a1 - b, y1 - b, y1, c0, c1);
	const float cm = (c0 + c1) * 0.5f, am = (a0 + a1) * 0.5f;
	box(am - 0.02f, am + 0.02f, y0 + b, y1 - b, cm - 0.02f, cm + 0.02f);
	if (extSide > 0) box(a0 - 0.05f, a1 + 0.05f, y0 - 0.04f, y0, c1, c1 + 0.08f);
	else box(a0 - 0.05f, a1 + 0.05f, y0 - 0.04f, y0, c0 - 0.08f, c0);
	const V3 n = (axis == 0) ? V3{ 0, 0, (float)extSide } : V3{ (float)extSide, 0, 0 };
	auto P = [&](float a, float y) { return (axis == 0) ? V3{ a, y, cm } : V3{ cm, y, a }; };
	const V3 q[4] = { P(a0 + b, y0 + b), P(a1 - b, y0 + b), P(a1 - b, y1 - b), P(a0 + b, y1 - b) };
	const int idx[6] = { 0, 1, 2, 0, 2, 3 };
	for (int i : idx) pushV(g, q[i], n, cPane);
}

static void hzChair(Verts& v, float cx, float cz, float s, float yb) {
	const V3 cWood = { 0.62f, 0.45f, 0.28f }, cWoodD = { 0.42f, 0.29f, 0.18f };
	addBoxR(v, cx - 0.21f, cx + 0.21f, yb + 0.42f, yb + 0.47f, cz - 0.21f, cz + 0.21f, cWood);
	for (int a = -1; a <= 1; a += 2) for (int b = -1; b <= 1; b += 2)
		addBoxR(v, cx + a * 0.18f - 0.018f, cx + a * 0.18f + 0.018f, yb, yb + 0.42f, cz + b * 0.18f - 0.018f, cz + b * 0.18f + 0.018f, cWoodD);
	const float zb = cz - s * 0.19f;
	addBoxR(v, cx - 0.21f, cx + 0.21f, yb + 0.47f, yb + 0.92f, zb - 0.02f, zb + 0.02f, cWood);
}

static void hzPot(Verts& v, V3 c, float r, float h) {
	const V3 steel = { 0.62f, 0.64f, 0.67f }, inner = { 0.48f, 0.50f, 0.53f };
	const int N = 16;
	const float wt = 0.012f;
	for (int i = 0; i < N; i++) {
		const float a0 = 2.0f * PI * i / N, a1 = 2.0f * PI * (i + 1) / N;
		const V3 d0 = { cosf(a0), 0, sinf(a0) }, d1 = { cosf(a1), 0, sinf(a1) };

		pushV(v, c + d0 * r, d0, steel); pushV(v, c + d1 * r, d1, steel); pushV(v, c + d1 * r + V3{ 0, h, 0 }, d1, steel);
		pushV(v, c + d0 * r, d0, steel); pushV(v, c + d1 * r + V3{ 0, h, 0 }, d1, steel); pushV(v, c + d0 * r + V3{ 0, h, 0 }, d0, steel);

		const V3 m0 = d0 * -1.0f, m1 = d1 * -1.0f;
		pushV(v, c + d0 * (r - wt), m0, inner); pushV(v, c + d1 * (r - wt), m1, inner); pushV(v, c + d1 * (r - wt) + V3{ 0, h, 0 }, m1, inner);
		pushV(v, c + d0 * (r - wt), m0, inner); pushV(v, c + d1 * (r - wt) + V3{ 0, h, 0 }, m1, inner); pushV(v, c + d0 * (r - wt) + V3{ 0, h, 0 }, m0, inner);

		const V3 up = { 0, 1, 0 }, dn = { 0, -1, 0 };
		pushV(v, c + d0 * r + V3{ 0, h, 0 }, up, steel); pushV(v, c + d1 * r + V3{ 0, h, 0 }, up, steel); pushV(v, c + d1 * (r - wt) + V3{ 0, h, 0 }, up, steel);
		pushV(v, c + d0 * r + V3{ 0, h, 0 }, up, steel); pushV(v, c + d1 * (r - wt) + V3{ 0, h, 0 }, up, steel); pushV(v, c + d0 * (r - wt) + V3{ 0, h, 0 }, up, steel);

		pushV(v, c, dn, steel); pushV(v, c + d1 * r, dn, steel); pushV(v, c + d0 * r, dn, steel);
		const V3 sc = { 0.80f, 0.54f, 0.20f }; const V3 sl = c + V3{ 0, h * 0.78f, 0 };
		pushV(v, sl, up, sc); pushV(v, sl + d0 * (r - wt), up, sc); pushV(v, sl + d1 * (r - wt), up, sc);
	}

	const V3 cc[7] = { { 0.90f, 0.45f, 0.10f }, { 0.88f, 0.78f, 0.52f }, { 0.55f, 0.75f, 0.30f }, { 0.55f, 0.32f, 0.20f }, { 0.90f, 0.46f, 0.12f }, { 0.86f, 0.76f, 0.50f }, { 0.60f, 0.80f, 0.32f } };
	const float px[7] = { -0.05f, 0.04f, 0.00f, -0.02f, 0.06f, -0.07f, 0.03f }, pz[7] = { 0.03f, -0.04f, 0.07f, -0.06f, 0.05f, -0.03f, 0.00f };
	for (int k = 0; k < 7; k++)
		addBlob(v, c + V3{ px[k], h * 0.80f, pz[k] }, { 0.032f, 0.022f, 0.032f }, (float)k * 1.7f, cc[k]);

	addBoxR(v, c.x - r - 0.05f, c.x - r, c.y + h - 0.04f, c.y + h - 0.02f, c.z - 0.02f, c.z + 0.02f, { 0.15f, 0.15f, 0.17f });
	addBoxR(v, c.x + r, c.x + r + 0.05f, c.y + h - 0.04f, c.y + h - 0.02f, c.z - 0.02f, c.z + 0.02f, { 0.15f, 0.15f, 0.17f });
}

static void buildHouseGeometry(HouseVerts& H) {
	Verts v, g;
	g_curE = 0.0f;
	auto B = [&](float x0, float x1, float y0, float y1, float z0, float z1, V3 c) { addBoxR(v, x0, x1, y0, y1, z0, z1, c); };
	const V3 cWE = { 0.93f, 0.90f, 0.82f }, cWI = { 0.96f, 0.94f, 0.89f }, cFound = { 0.55f, 0.55f, 0.53f };
	const V3 cFl1 = { 0.58f, 0.41f, 0.26f }, cFl2 = { 0.66f, 0.50f, 0.33f }, cCeil = { 0.97f, 0.97f, 0.95f };
	const V3 cWood = { 0.62f, 0.45f, 0.28f }, cWoodD = { 0.42f, 0.29f, 0.18f }, cWoodL = { 0.78f, 0.66f, 0.50f };
	const V3 cSteel = { 0.72f, 0.74f, 0.77f }, cDark = { 0.12f, 0.12f, 0.14f }, cRoof = { 0.27f, 0.30f, 0.38f }, cFrame = { 0.97f, 0.97f, 0.95f };


	B(-4.6f, 4.6f, -0.6f, 0.0f, 3.8f, 4.1f, cFound); B(-4.6f, 4.6f, -0.6f, 0.0f, -4.1f, -3.8f, cFound);
	B(-4.6f, -4.3f, -0.6f, 0.0f, -3.8f, 3.8f, cFound); B(4.3f, 4.6f, -0.6f, 0.0f, -3.8f, 3.8f, cFound);
	B(-4.3f, 4.3f, -0.3f, 0.0f, -3.8f, 3.8f, cFl1);
	B(-4.3f, 3.1f, 2.5f, 2.65f, -3.8f, 3.8f, cCeil); B(-4.3f, 3.1f, 2.65f, 2.75f, -3.8f, 3.8f, cFl2);
	B(3.1f, 4.3f, 2.5f, 2.65f, -3.8f, -2.3f, cCeil); B(3.1f, 4.3f, 2.65f, 2.75f, -3.8f, -2.3f, cFl2);
	B(3.1f, 4.3f, 2.5f, 2.65f, 0.3f, 3.8f, cCeil);   B(3.1f, 4.3f, 2.65f, 2.75f, 0.3f, 3.8f, cFl2);
	B(-4.3f, 4.3f, 5.15f, 5.4f, -3.8f, 3.8f, cCeil);

	B(2.4f, 4.0f, -0.6f, -0.07f, 4.0f, 4.9f, cFound);


	struct WinDef { int axis; float c0, c1; int ext; float a0, a1, y0, y1; };
	const WinDef wins[] = {
		{ 0, 3.8f, 4.0f, 1, -3.2f, -1.6f, 0.9f, 2.0f }, { 0, 3.8f, 4.0f, 1, -0.8f, 0.8f, 0.9f, 2.0f },
		{ 0, 3.8f, 4.0f, 1, -3.2f, -1.6f, 3.65f, 4.75f }, { 0, 3.8f, 4.0f, 1, -0.8f, 0.8f, 3.65f, 4.75f },
		{ 0, -4.0f, -3.8f, -1, -2.95f, -2.05f, 1.15f, 2.05f }, { 0, -4.0f, -3.8f, -1, 1.0f, 2.4f, 0.9f, 2.0f }, { 0, -4.0f, -3.8f, -1, -1.0f, 0.6f, 3.65f, 4.75f },
		{ 1, -4.5f, -4.3f, -1, 0.6f, 2.2f, 0.9f, 2.0f }, { 1, -4.5f, -4.3f, -1, -0.2f, 1.4f, 3.65f, 4.75f },
		{ 1, 4.3f, 4.5f, 1, 1.0f, 2.4f, 0.9f, 2.0f }, { 1, 4.3f, 4.5f, 1, 1.0f, 2.4f, 3.65f, 4.75f },
	};
	struct WallDef { int axis; float c0, c1; int ext; float a0, a1; };
	const WallDef walls[4] = { { 0, 3.8f, 4.0f, 1, -4.5f, 4.5f }, { 0, -4.0f, -3.8f, -1, -4.5f, 4.5f }, { 1, -4.5f, -4.3f, -1, -3.8f, 3.8f }, { 1, 4.3f, 4.5f, 1, -3.8f, 3.8f } };
	for (int w = 0; w < 4; w++) {
		const WallDef& W = walls[w];
		std::vector<WOpen> o1, o2;
		if (w == 0) o1.push_back({ 2.70f, 3.70f, 0.0f, 2.10f });
		for (const WinDef& d : wins) if (d.axis == W.axis && d.c0 == W.c0) (d.y0 < 2.5f ? o1 : o2).push_back({ d.a0, d.a1, d.y0, d.y1 });
		hzWall(v, W.axis, W.c0, W.c1, W.ext, W.a0, W.a1, 0.0f, 2.5f, o1, cWE, cWI);
		hzWall(v, W.axis, W.c0, W.c1, W.ext, W.a0, W.a1, 2.5f, 2.75f, {}, cWE, cWI);
		hzWall(v, W.axis, W.c0, W.c1, W.ext, W.a0, W.a1, 2.75f, 5.4f, o2, cWE, cWI);
	}
	for (const WinDef& d : wins) hzWindow(v, g, d.axis, d.c0, d.c1, d.ext, d.a0, d.a1, d.y0, d.y1);

	B(2.66f, 2.70f, 0.0f, 2.14f, 3.8f, 4.0f, cFrame); B(3.70f, 3.74f, 0.0f, 2.14f, 3.8f, 4.0f, cFrame); B(2.66f, 3.74f, 2.10f, 2.14f, 3.8f, 4.0f, cFrame);

	hzWall(v, 1, 3.0f, 3.1f, 0, -3.8f, 0.4f, 2.75f, 5.15f, { { -3.55f, -2.65f, 2.75f, 4.85f } }, cWI, cWI);
	hzWall(v, 0, 0.3f, 0.4f, 0, 3.1f, 4.3f, 2.75f, 5.15f, {}, cWI, cWI);
	B(3.0f, 3.1f, 2.75f, 4.89f, -3.59f, -3.55f, cFrame); B(3.0f, 3.1f, 2.75f, 4.89f, -2.65f, -2.61f, cFrame); B(3.0f, 3.1f, 4.85f, 4.89f, -3.59f, -2.61f, cFrame);


	{
		const float ang = 0.52f, ca = cosf(ang), sa = sinf(ang), ta = sa / ca, ov = 0.55f, th = 0.07f;
		const float yTop0 = 5.45f + 4.0f * ta, zE = 4.0f + ov, Ls = zE / ca;
		for (int sg = -1; sg <= 1; sg += 2) {
			const V3 f = { 0.0f, -sa, sg * ca }, u = { 0.0f, ca, sg * sa };
			const V3 midTop = { 0.0f, yTop0 - 0.5f * zE * ta, sg * 0.5f * zE };
			addBox(v, midTop - u * th, { 1, 0, 0 }, u, f, { 4.9f, th, Ls * 0.5f }, cRoof);
		}
		B(-4.9f, 4.9f, yTop0, yTop0 + 0.08f, -0.12f, 0.12f, cRoof * 0.8f);
		const float yApex = yTop0 - 0.085f;
		for (int sx = -1; sx <= 1; sx += 2)
			addTri(v, { sx * 4.5f, 5.4f, -4.0f }, { sx * 4.5f, 5.4f, 4.0f }, { sx * 4.5f, yApex, 0.0f }, cWE, { 0.0f, 5.0f, 0.0f });
	}


	for (int k = 1; k <= 10; k++)
		B(3.1f, 4.3f, 0.0f, 0.25f * k, 0.4f - 0.27f * k, 0.4f - 0.27f * (k - 1), (k & 1) ? cWoodL : cWoodL * 0.92f);
	{
		const V3 P0 = { 3.1f, 1.15f, 0.4f }, P1 = { 3.1f, 3.40f, -2.3f };
		const V3 d = P1 - P0; const float len = sqrtf(dot(d, d)); const V3 f = d * (1.0f / len);
		const V3 u = norm(cross({ 1, 0, 0 }, f));
		addBox(v, (P0 + P1) * 0.5f, { 1, 0, 0 }, u, f, { 0.025f, 0.025f, len * 0.5f }, cWoodD);
		for (int k : { 2, 6, 10 }) { const float zc = 0.4f - 0.27f * (k - 0.5f); B(3.08f, 3.12f, 0.25f * k, 0.25f * k + 0.92f, zc - 0.02f, zc + 0.02f, cWoodD); }
	}



	{
		const V3 cB = { 0.88f, 0.89f, 0.91f }, cI = { 0.96f, 0.97f, 0.98f }, cSh = { 0.72f, 0.86f, 0.92f };
		B(-4.25f, -3.60f, 0.0f, 1.80f, -3.78f, -3.76f, cB); B(-4.25f, -3.60f, 0.04f, 1.76f, -3.76f, -3.74f, cI);
		B(-4.25f, -4.23f, 0.0f, 1.80f, -3.78f, -3.16f, cB); B(-4.23f, -4.21f, 0.04f, 1.76f, -3.76f, -3.16f, cI);
		B(-3.62f, -3.60f, 0.0f, 1.80f, -3.78f, -3.16f, cB); B(-3.64f, -3.62f, 0.04f, 1.76f, -3.76f, -3.16f, cI);
		B(-4.25f, -3.60f, 1.78f, 1.80f, -3.78f, -3.16f, cB); B(-4.21f, -3.64f, 1.76f, 1.78f, -3.76f, -3.16f, cI);
		B(-4.25f, -3.60f, 0.0f, 0.02f, -3.78f, -3.16f, cB); B(-4.21f, -3.64f, 0.02f, 0.04f, -3.76f, -3.16f, cI);
		B(-4.21f, -3.64f, 1.18f, 1.22f, -3.76f, -3.16f, cI);
		B(-4.21f, -3.64f, 0.50f, 0.52f, -3.74f, -3.20f, cSh); B(-4.21f, -3.64f, 0.86f, 0.88f, -3.74f, -3.20f, cSh);
		B(-4.18f, -4.09f, 0.52f, 0.76f, -3.62f, -3.52f, { 0.95f, 0.95f, 0.98f }); B(-4.17f, -4.10f, 0.76f, 0.80f, -3.60f, -3.54f, { 0.20f, 0.40f, 0.85f });
		B(-3.98f, -3.89f, 0.52f, 0.72f, -3.64f, -3.54f, { 0.95f, 0.55f, 0.10f });
		addBlob(v, { -4.12f, 0.56f, -3.38f }, { 0.04f, 0.04f, 0.04f }, 1.0f, { 0.85f, 0.15f, 0.12f });
		addBlob(v, { -4.03f, 0.56f, -3.36f }, { 0.04f, 0.04f, 0.04f }, 2.0f, { 0.88f, 0.20f, 0.14f });
		B(-4.17f, -3.95f, 0.88f, 0.94f, -3.62f, -3.42f, { 0.86f, 0.76f, 0.56f });
		B(-3.86f, -3.76f, 0.88f, 1.00f, -3.64f, -3.54f, { 0.78f, 0.20f, 0.16f });
		B(-4.19f, -3.66f, 0.04f, 0.30f, -3.74f, -3.34f, { 0.36f, 0.66f, 0.32f });
		B(-4.17f, -3.70f, 1.22f, 1.28f, -3.72f, -3.40f, { 0.70f, 0.85f, 0.95f });
		B(-4.17f, -3.88f, 1.28f, 1.50f, -3.70f, -3.50f, { 0.30f, 0.50f, 0.80f });
		B(-3.82f, -3.70f, 1.28f, 1.44f, -3.70f, -3.52f, { 0.90f, 0.50f, 0.50f });
	}
	B(-3.6f, -2.1f, 0.0f, 0.88f, -3.8f, -3.15f, { 0.78f, 0.68f, 0.52f });
	B(-1.2f, -0.6f, 0.0f, 0.88f, -3.8f, -3.15f, { 0.78f, 0.68f, 0.52f });
	B(-3.6f, -2.1f, 0.88f, 0.92f, -3.8f, -3.10f, { 0.85f, 0.85f, 0.83f }); B(-1.2f, -0.6f, 0.88f, 0.92f, -3.8f, -3.10f, { 0.85f, 0.85f, 0.83f });
	B(-2.85f, -2.25f, 0.92f, 0.935f, -3.72f, -3.30f, cSteel);
	B(-2.57f, -2.53f, 0.935f, 1.10f, -3.75f, -3.71f, cSteel); B(-2.57f, -2.53f, 1.06f, 1.10f, -3.71f, -3.55f, cSteel);
	B(-2.1f, -1.2f, 0.0f, 0.88f, -3.8f, -3.15f, { 0.70f, 0.72f, 0.75f });
	B(-2.1f, -1.2f, 0.88f, 0.92f, -3.8f, -3.10f, { 0.10f, 0.11f, 0.13f });
	B(-2.0f, -1.3f, 0.25f, 0.70f, -3.15f, -3.14f, cDark);
	for (int bnr = 0; bnr < 2; bnr++) {
		const float bx = bnr ? -1.45f : -1.85f;
		B(bx - 0.11f, bx + 0.11f, 0.92f, 0.93f, -3.61f, -3.39f, { 0.15f, 0.15f, 0.17f });
		B(bx - 0.13f, bx + 0.13f, 0.93f, 0.96f, -3.512f, -3.488f, { 0.20f, 0.20f, 0.22f });
		B(bx - 0.012f, bx + 0.012f, 0.93f, 0.96f, -3.63f, -3.37f, { 0.20f, 0.20f, 0.22f });
		B(bx - 0.025f, bx + 0.025f, 0.76f, 0.82f, -3.15f, -3.10f, { 0.08f, 0.08f, 0.09f });
	}
	B(-2.1f, -1.2f, 1.95f, 2.12f, -3.8f, -3.3f, cSteel); B(-1.85f, -1.45f, 2.12f, 2.5f, -3.8f, -3.5f, cSteel);
	hzPot(v, { -1.85f, 0.96f, -3.5f }, 0.14f, 0.13f);


	B(-1.5f, 2.3f, 0.0f, 0.012f, -1.3f, 2.1f, { 0.55f, 0.30f, 0.30f });
	B(-0.4f, 1.2f, 0.70f, 0.75f, -0.2f, 0.7f, cWood);
	for (int a = -1; a <= 1; a += 2) for (int b = -1; b <= 1; b += 2) B(0.4f + a * 0.74f - 0.03f, 0.4f + a * 0.74f + 0.03f, 0.0f, 0.70f, 0.25f + b * 0.44f - 0.03f, 0.25f + b * 0.44f + 0.03f, cWoodD);
	for (int k = 0; k < 4; k++) hzChair(v, H_CHAIRS[k].x, H_CHAIRS[k].z, H_CHAIRS[k].s, H_CHAIRS[k].yb);
	B(-4.28f, -3.8f, 0.0f, 0.75f, -0.4f, 1.0f, cWood);
	B(-4.1f, -3.9f, 0.75f, 0.95f, 2.9f, 3.1f, { 0.60f, 0.35f, 0.22f });
	addBlob(v, { -4.0f, 1.25f, 3.0f }, { 0.28f, 0.30f, 0.28f }, 1.0f, { 0.22f, 0.50f, 0.22f });
	addBlob(v, { -3.9f, 1.55f, 3.05f }, { 0.20f, 0.22f, 0.20f }, 2.0f, { 0.26f, 0.56f, 0.24f });

	B(2.19f, 2.31f, 1.14f, 1.26f, 3.77f, 3.8f, { 0.97f, 0.97f, 0.95f }); B(2.22f, 2.28f, 1.17f, 1.23f, 3.75f, 3.77f, { 0.85f, 0.82f, 0.70f });

	B(-0.05f, 0.45f, 2.46f, 2.5f, -0.05f, 0.45f, cCeil);


	B(-4.25f, -3.10f, 2.75f, 3.15f, -3.75f, -1.65f, cWoodD);
	B(-4.22f, -3.13f, 3.15f, 3.32f, -3.72f, -1.68f, { 0.88f, 0.90f, 0.95f });
	B(-4.22f, -3.13f, 3.32f, 3.36f, -2.85f, -1.68f, { 0.33f, 0.50f, 0.72f });
	B(-4.05f, -3.30f, 3.32f, 3.42f, -3.68f, -3.18f, { 0.97f, 0.97f, 0.98f });
	B(-4.25f, -3.10f, 3.15f, 3.90f, -3.80f, -3.72f, cWoodD);
	B(-3.3f, -1.9f, 3.43f, 3.48f, 2.9f, 3.6f, cWood);
	for (int a = -1; a <= 1; a += 2) for (int b = -1; b <= 1; b += 2) B(-2.6f + a * 0.65f - 0.03f, -2.6f + a * 0.65f + 0.03f, 2.75f, 3.43f, 3.25f + b * 0.3f - 0.03f, 3.25f + b * 0.3f + 0.03f, cWoodD);
	B(-3.3f, -1.9f, 3.0f, 3.43f, 3.55f, 3.58f, cWoodD);
	B(-2.85f, -2.5f, 3.48f, 3.50f, 3.10f, 3.40f, cDark); B(-2.85f, -2.5f, 3.50f, 3.72f, 3.38f, 3.40f, cDark);
	B(-3.25f, -3.05f, 3.48f, 3.55f, 3.2f, 3.5f, { 0.70f, 0.20f, 0.20f });
	hzChair(v, H_CHAIRS[4].x, H_CHAIRS[4].z, H_CHAIRS[4].s, H_CHAIRS[4].yb);
	const V3 cLampD = { 0.12f, 0.15f, 0.14f };
	B(-2.05f, -1.93f, 3.48f, 3.51f, 3.24f, 3.36f, cLampD); B(-2.00f, -1.98f, 3.51f, 3.82f, 3.29f, 3.31f, cLampD);
	B(-2.30f, -1.98f, 3.80f, 3.83f, 3.29f, 3.31f, cLampD); B(-2.42f, -2.18f, 3.70f, 3.80f, 3.18f, 3.42f, { 0.15f, 0.38f, 0.28f });
	B(-2.6f, -0.4f, 2.75f, 2.762f, -1.5f, 0.4f, { 0.30f, 0.38f, 0.55f });
	B(-4.25f, -3.75f, 2.75f, 3.55f, 0.2f, 1.6f, cWood);
	B(2.97f, 3.0f, 3.89f, 4.01f, -2.46f, -2.34f, { 0.97f, 0.97f, 0.95f }); B(2.95f, 2.97f, 3.92f, 3.98f, -2.43f, -2.37f, { 0.85f, 0.82f, 0.70f });
	B(-0.85f, -0.35f, 5.11f, 5.15f, -0.05f, 0.45f, cCeil);


	for (Vtx& x : v) { x.p = hzW(x.p); x.n = hzWDir(x.n); }
	H.stat = v; H.glass = g;


	for (int d = 0; d < NHD; d++) {
		Verts dv;
		const HDoorDef& D = H_DOORS[d];
		if (d >= 2) {
			addBoxR(dv, 0.0f, D.w, 0.0f, D.h, -0.03f, 0.03f, { 0.88f, 0.89f, 0.91f });
			const float hy0 = (d == 2) ? 0.70f : 0.04f, hy1 = (d == 2) ? 1.08f : 0.40f;
			addBoxR(dv, D.w - 0.075f, D.w - 0.045f, hy0, hy1, 0.03f, 0.05f, { 0.62f, 0.64f, 0.67f });
			H.door[d] = dv; continue;
		}
		const V3 col = d == 0 ? V3{ 0.45f, 0.28f, 0.16f } : V3{ 0.86f, 0.83f, 0.76f };
		addBoxR(dv, 0.0f, D.w, 0.0f, D.h, -0.022f, 0.022f, col);
		addBoxR(dv, D.w - 0.14f, D.w - 0.08f, 0.95f, 1.01f, -0.065f, 0.065f, { 0.85f, 0.72f, 0.30f });
		if (d == 0) for (int sg = -1; sg <= 1; sg += 2) addBoxR(dv, 0.12f, D.w - 0.12f, 0.20f, 1.90f, sg > 0 ? 0.022f : -0.026f, sg > 0 ? 0.026f : -0.022f, col * 0.82f);
		H.door[d] = dv;
	}
	g_curE = 2.0f;
	{ Verts a; addBoxR(a, 0.0f, 0.4f, 2.43f, 2.46f, 0.0f, 0.4f, { 1.0f, 0.97f, 0.88f }); H.glow[0] = a; }
	{ Verts a; addBoxR(a, -0.8f, -0.4f, 5.08f, 5.11f, 0.0f, 0.4f, { 1.0f, 0.97f, 0.88f }); H.glow[1] = a; }
	{ Verts a; addBoxR(a, -2.36f, -2.24f, 3.66f, 3.70f, 3.24f, 3.36f, { 1.0f, 0.90f, 0.70f }); H.glow[2] = a; }
	{ Verts a; for (int i = 0; i < 8; i++) { const float an = 2.0f * PI * i / 8; addConeTier(a, { cosf(an) * 0.07f, 0.0f, sinf(an) * 0.07f }, an, 0.017f, 0.05f, { 0.40f, 0.62f, 1.0f }); } H.flame = a; }

	{

		const float PW = PC_X1 - PC_X0, PH = PC_Y1 - PC_Y0;

		auto X = [&](float u) { return PC_X0 + u * PW; };

		auto Y = [&](float t) { return PC_Y0 + t * PH; };

		auto ZF = [&](int layer) { return PC_ZB - 0.0006f * (float)(layer + 1); };

		auto rect = [&](Verts& q, float u0, float u1, float t0, float t1, int layer, V3 c) { addBoxR(q, X(u0), X(u1), Y(t0), Y(t1), ZF(layer), PC_ZB, c); };

		auto disc = [&](Verts& q, float cx, float cy, float r, int layer, V3 c) {

			const V3 nn = { 0.0f, 0.0f, -1.0f }; const float z = ZF(layer);

			for (int i = 0; i < 24; i++) {

				const float t0 = 2.0f * PI * i / 24, t1 = 2.0f * PI * (i + 1) / 24;

				pushV(q, { cx, cy, z }, nn, c); pushV(q, { cx + r * cosf(t0), cy + r * sinf(t0), z }, nn, c); pushV(q, { cx + r * cosf(t1), cy + r * sinf(t1), z }, nn, c);

			}

			};

		Verts sc;

		for (int i = 0; i < 12; i++) {

			const float t0 = i / 12.0f, t1 = (i + 1) / 12.0f, k = powf((t0 + t1) * 0.5f, 0.8f);

			rect(sc, 0.0f, 1.0f, t0, t1, 0, { 0.95f + (0.06f - 0.95f) * k, 0.45f + (0.07f - 0.45f) * k, 0.30f + (0.28f - 0.30f) * k });

		}

		rect(sc, 0.0f, 1.0f, 0.0f, 0.10f, 1, { 0.08f, 0.09f, 0.14f });

		rect(sc, 0.02f, 0.08f, 0.02f, 0.08f, 2, { 0.25f, 0.55f, 0.95f });

		const V3 ic[4] = { { 0.98f, 0.60f, 0.15f }, { 0.35f, 0.85f, 0.45f }, { 0.95f, 0.40f, 0.65f }, { 0.98f, 0.88f, 0.30f } };

		for (int k = 0; k < 4; k++) rect(sc, 0.12f + 0.07f * k, 0.17f + 0.07f * k, 0.025f, 0.075f, 2, ic[k]);

		rect(sc, 0.86f, 0.98f, 0.03f, 0.07f, 2, { 0.85f, 0.88f, 0.95f });

		rect(sc, 0.06f, 0.60f, 0.20f, 0.90f, 2, { 0.93f, 0.95f, 0.99f });

		rect(sc, 0.06f, 0.60f, 0.80f, 0.90f, 3, { 0.20f, 0.45f, 0.88f });

		disc(sc, X(0.09f), Y(0.85f), 0.0045f, 4, { 0.95f, 0.35f, 0.35f });

		disc(sc, X(0.12f), Y(0.85f), 0.0045f, 4, { 0.98f, 0.80f, 0.25f });

		disc(sc, X(0.15f), Y(0.85f), 0.0045f, 4, { 0.35f, 0.85f, 0.40f });

		rect(sc, 0.28f, 0.52f, 0.835f, 0.865f, 4, { 0.75f, 0.85f, 1.00f });

		rect(sc, 0.09f, 0.57f, 0.25f, 0.262f, 3, { 0.50f, 0.50f, 0.55f });

		for (int g2 = 0; g2 < 3; g2++) rect(sc, 0.09f, 0.57f, 0.40f + 0.14f * g2, 0.405f + 0.14f * g2, 3, { 0.80f, 0.82f, 0.88f });

		rect(sc, 0.64f, 0.96f, 0.20f, 0.90f, 2, { 0.10f, 0.12f, 0.20f });

		disc(sc, X(0.80f), Y(0.58f), 0.034f, 3, { 0.25f, 0.85f, 0.75f });

		disc(sc, X(0.80f), Y(0.58f), 0.026f, 4, { 0.06f, 0.08f, 0.18f });

		rect(sc, 0.68f, 0.92f, 0.30f, 0.35f, 3, { 0.20f, 0.22f, 0.30f });

		rect(sc, 0.68f, 0.90f, 0.80f, 0.82f, 3, { 0.50f, 0.60f, 0.80f });

		rect(sc, 0.68f, 0.84f, 0.85f, 0.87f, 3, { 0.50f, 0.60f, 0.80f });

		H.pcStatic = sc;

		const V3 bc[6] = { { 0.95f, 0.35f, 0.35f }, { 0.98f, 0.65f, 0.20f }, { 0.95f, 0.90f, 0.30f }, { 0.40f, 0.85f, 0.40f }, { 0.30f, 0.60f, 0.95f }, { 0.65f, 0.45f, 0.90f } };

		for (int k = 0; k < 6; k++) { Verts b; addBoxR(b, -0.5f, 0.5f, 0.0f, 1.0f, ZF(4), PC_ZB, bc[k]); H.pcBar[k] = b; }

		{ Verts b; addBoxR(b, -0.0015f, 0.0015f, 0.0f, 0.027f, ZF(5), PC_ZB, { 0.98f, 0.98f, 1.00f }); H.pcNeedle = b; }

		{ Verts b; addBoxR(b, 0.0f, 1.0f, 0.0f, 1.0f, ZF(4), PC_ZB, { 0.30f, 0.90f, 0.50f }); H.pcFill = b; }

	}
	g_curE = 1.0f;
	{ Verts a; addBlob(a, { 0, 0, 0 }, { 1, 1, 1 }, 0.0f, { 0.93f, 0.93f, 0.95f }); H.sphere = a; }
	{ Verts a; addBoxR(a, -0.009f, 0.009f, 0.0f, 0.125f, -0.009f, 0.009f, { 0.80f, 0.90f, 1.00f }); H.water = a; }
	g_curE = 0.0f;
}


static Verts hzDoorWorld(int d, float ang) {
	Verts out = g_hvx.door[d];
	const HDoorDef& D = H_DOORS[d];
	const V3 Rd = D.closedDir * cosf(ang) + D.openDir * sinf(ang), Fd = cross(Rd, { 0, 1, 0 });
	for (Vtx& x : out) {
		const V3 lp = D.hinge + Rd * x.p.x + V3{ 0, 1, 0 } * x.p.y + Fd * x.p.z;
		const V3 ln = Rd * x.n.x + V3{ 0, 1, 0 } * x.n.y + Fd * x.n.z;
		x.p = hzW(lp); x.n = hzWDir(ln);
	}
	return out;
}


static Verts g_bldInter;
static Verts g_bldGlass;
static Verts g_colExtra;

static void addBoxShade(Verts& v, V3 c, V3 r, V3 u, V3 f, V3 h, V3 col, float lum) {
	V3 ax[3] = { r, u, f }; float hs[3] = { h.x, h.y, h.z };
	static const float fac[3][2] = { { 0.78f, 0.88f }, { 0.55f, 1.0f }, { 0.70f, 0.93f } };
	for (int k = 0; k < 3; k++) for (int sg = -1; sg <= 1; sg += 2) {
		if (k == 1 && sg < 0) continue;
		int a = (k + 1) % 3, b = (k + 2) % 3;
		V3 n = ax[k] * (float)sg, ca = c + n * hs[k];
		V3 A = ax[a] * hs[a], B = ax[b] * hs[b];
		V3 q[4] = { ca - A - B, ca + A - B, ca + A + B, ca - A + B };
		const V3 cc = col * (fac[k][sg > 0 ? 1 : 0] * lum);
		int idx[6] = { 0, 1, 2, 0, 2, 3 };
		for (int i : idx) v.push_back({ q[i], n, cc, 2.0f });
	}
}
static void addQuadV(Verts& v, V3 a, V3 b, V3 c, V3 d, V3 n, V3 col, float e) {
	const V3 p[6] = { a, b, c, a, c, d };
	for (int i = 0; i < 6; i++) v.push_back({ p[i], n, col, e });
}

static void addBuilding(Verts& shell, V3 p, float yaw, float w, float d, int nF, V3 wallCol, V3 bandCol, float drop, std::mt19937& rng) {
	auto rf = [&](float a, float b) { return a + (b - a) * (float)(rng() / 4294967296.0); };
	const float cy = cosf(yaw), sy = sinf(yaw);
	const V3 R = { cy, 0, -sy }, Z = { sy, 0, cy }, U = { 0, 1, 0 };
	const float FH = 3.4f, t = 0.40f, y00 = 0.15f;
	const float Ht = nF * FH;
	auto W = [&](float lx, float ly, float lz) { return p + R * lx + U * (ly + y00) + Z * lz; };
	auto box = [&](float lx, float ly, float lz, float hx, float hy, float hz, V3 col) { addBox(shell, W(lx, ly, lz), R, U, Z, { hx, hy, hz }, col); };

	box(0, -(drop + 1.2f) * 0.5f - 0.0f + 0.0f, 0, w * 0.5f + 0.15f, (drop + 1.2f) * 0.5f, d * 0.5f + 0.15f, V3{ 0.52f, 0.52f, 0.50f });
	box(0, Ht + 0.12f, 0, w * 0.5f + 0.05f, 0.18f, d * 0.5f + 0.05f, bandCol * 0.8f);
	for (int sd = 0; sd < 4; sd++) {
		const bool zs = sd < 2; const float sg = (sd & 1) ? -1.0f : 1.0f;
		if (zs) box(0, Ht + 0.55f, sg * (d * 0.5f - 0.1f), w * 0.5f, 0.35f, 0.10f, bandCol);
		else box(sg * (w * 0.5f - 0.1f), Ht + 0.55f, 0, 0.10f, 0.35f, d * 0.5f, bandCol);
	}
	const int nAC = 2 + (int)(rng() % 3);
	for (int i = 0; i < nAC; i++) box(rf(-w * 0.35f, w * 0.35f), Ht + 0.30f + 0.5f, rf(-d * 0.3f, d * 0.3f), rf(0.5f, 0.9f), 0.5f, rf(0.4f, 0.7f), V3{ 0.70f, 0.72f, 0.72f });
	box(w * 0.28f, Ht + 1.6f, -d * 0.25f, 1.4f, 1.3f, 1.4f, wallCol * 0.9f);
	if (nF >= 6) { box(-w * 0.3f, Ht + 2.3f, d * 0.2f, 0.9f, 0.9f, 0.9f, V3{ 0.55f, 0.58f, 0.62f }); for (int sx = -1; sx <= 1; sx += 2) for (int sz = -1; sz <= 1; sz += 2) box(-w * 0.3f + 0.7f * sx, Ht + 0.9f, d * 0.2f + 0.7f * sz, 0.05f, 0.6f, 0.05f, V3{ 0.3f, 0.3f, 0.3f }); }


	std::vector<float> wy0(nF), wy1(nF);
	for (int f = 0; f < nF; f++) { wy0[f] = f * FH + ((f == 0) ? 0.35f : 0.95f); wy1[f] = f * FH + ((f == 0) ? 2.75f : 2.65f); }
	const V3 nrm[4] = { Z, Z * -1.0f, R, R * -1.0f };
	const V3 ax[4] = { R, R, Z, Z };
	const float Ls[4] = { w, w, d, d };
	const float off[4] = { d * 0.5f, d * 0.5f, w * 0.5f, w * 0.5f };
	const float offS[4] = { 1, -1, 1, -1 };
	const int building_bias = (int)(rng() % 3);
	const V3 glassC = { 0.50f, 0.66f, 0.76f };
	for (int sd = 0; sd < 4; sd++) {
		const V3 n = nrm[sd], a = ax[sd]; const float L = Ls[sd];
		const V3 fc = p + n * (off[sd] * offS[sd] * 0 + 0.0f);
		(void)fc;
		const V3 center = p + (sd < 2 ? Z * (off[sd] * offS[sd]) : R * (off[sd] * offS[sd])) + U * y00;
		const V3 nOut = (sd < 2) ? Z * offS[sd] : R * offS[sd];
		(void)n;
		const int N = std::max(2, (int)((L - 1.0f) / 3.5f));
		const float cell = L / N, ww = std::min(2.3f, cell * 0.66f);
		auto wallBox = [&](float xa0, float xa1, float y0, float y1, V3 col) {
			if (xa1 - xa0 < 0.02f || y1 - y0 < 0.02f) return;
			addBox(shell, center + a * ((xa0 + xa1) * 0.5f) + U * ((y0 + y1) * 0.5f) - nOut * (t * 0.5f), a, U, nOut, { (xa1 - xa0) * 0.5f, (y1 - y0) * 0.5f, t * 0.5f }, col);
			};

		wallBox(-L * 0.5f, L * 0.5f, 0.0f, wy0[0], bandCol * 0.95f);
		for (int f = 1; f < nF; f++) wallBox(-L * 0.5f, L * 0.5f, wy1[f - 1], wy0[f], bandCol);
		wallBox(-L * 0.5f, L * 0.5f, wy1[nF - 1], Ht + 0.5f, bandCol * 1.02f);

		for (int f = 0; f < nF; f++) {
			float prev = -L * 0.5f;
			for (int i = 0; i < N; i++) {
				const float xc = -L * 0.5f + (i + 0.5f) * cell;
				wallBox(prev, xc - ww * 0.5f, wy0[f], wy1[f], wallCol);
				prev = xc + ww * 0.5f;

				const V3 gc = center + a * xc + U * ((wy0[f] + wy1[f]) * 0.5f) - nOut * (t * 0.45f);
				const V3 A = a * (ww * 0.5f), B = U * ((wy1[f] - wy0[f]) * 0.5f);
				addQuadV(g_bldGlass, gc - A - B, gc + A - B, gc + A + B, gc - A + B, nOut, glassC, 0.0f);
				if (f <= 1) addQuadV(g_colExtra, gc - A - B, gc + A - B, gc + A + B, gc - A + B, nOut, glassC, 0.0f);
			}
			wallBox(prev, L * 0.5f, wy0[f], wy1[f], wallCol);
		}

		const float perp = (sd < 2) ? d : w;
		const float Dp = std::min(6.5f, perp * 0.5f - t - 0.1f);
		const V3 inward = nOut * -1.0f;
		const V3 base = center - nOut * t;
		auto IW = [&](float xa, float y, float dep) { return base + a * xa + U * y + inward * dep; };
		for (int f = 0; f < nF; f++) {
			const float y0 = f * FH + 0.02f;

			int cLo = 0, cHi = N;
			if (sd >= 2) {
				const float dz = std::min(6.5f, L * 0.5f - t - 0.1f);
				cLo = (int)ceilf((-L * 0.5f + t + dz + L * 0.5f) / cell - 1e-4f);
				cHi = (int)floorf((L * 0.5f - t - dz + L * 0.5f) / cell + 1e-4f);
			}
			int c0 = cLo;
			while (c0 < cHi) {
				int zc = 2 + (int)(rng() % 2); if (c0 + zc > cHi || cHi - (c0 + zc) == 1) zc = cHi - c0;
				const int c1 = c0 + zc;
				const float xa0 = -L * 0.5f + c0 * cell + (c0 == 0 ? t : 0.0f), xa1 = -L * 0.5f + c1 * cell - (c1 == N ? t : 0.0f);
				const bool lit = (rng() % 100) < 72;
				const float lum = lit ? 1.0f : 0.30f;
				int type = (f == 0) ? 2 : (building_bias == 0 ? ((rng() % 100) < 80 ? 0 : 1) : building_bias == 1 ? ((rng() % 100) < 80 ? 1 : 0) : (int)(rng() % 2));
				V3 wallI, floorI, ceilI = { 0.92f, 0.92f, 0.90f };
				if (type == 0) { wallI = { 0.86f, 0.87f, 0.84f }; floorI = { 0.42f, 0.45f, 0.50f }; }
				else if (type == 1) { wallI = { rf(0.78f, 0.92f), rf(0.72f, 0.86f), rf(0.62f, 0.78f) }; floorI = { 0.60f, 0.44f, 0.30f }; }
				else { wallI = { 0.90f, 0.90f, 0.88f }; floorI = { 0.74f, 0.71f, 0.66f }; }
				const float xm = (xa0 + xa1) * 0.5f, hxz = (xa1 - xa0) * 0.5f;
				const float yc = y0 + FH - 0.30f;

				addQuadV(g_bldInter, IW(xa0, y0, 0), IW(xa1, y0, 0), IW(xa1, y0, Dp), IW(xa0, y0, Dp), U, floorI * lum, 2.0f);
				addQuadV(g_bldInter, IW(xa0, yc, 0), IW(xa1, yc, 0), IW(xa1, yc, Dp), IW(xa0, yc, Dp), U * -1.0f, ceilI * lum, 2.0f);
				addQuadV(g_bldInter, IW(xa0, y0, Dp), IW(xa1, y0, Dp), IW(xa1, yc, Dp), IW(xa0, yc, Dp), nOut, wallI * (0.80f * lum), 2.0f);
				for (int e = 0; e < 2; e++) { if ((e == 0 && c0 == 0) || (e == 1 && c1 == N)) continue; const float xe = e ? xa1 : xa0; addQuadV(g_bldInter, IW(xe, y0, 0), IW(xe, y0, Dp), IW(xe, yc, Dp), IW(xe, yc, 0), a * (e ? -1.0f : 1.0f), wallI * (0.86f * lum), 2.0f); }

				for (float xx = xa0 + 1.4f; xx < xa1 - 0.6f; xx += 3.0f) addBoxShade(g_bldInter, IW(xx, yc - 0.02f, Dp * 0.5f), a, U, inward, { 0.55f, 0.02f, 0.22f }, lit ? V3{ 1.0f, 1.0f, 0.92f } : V3{ 0.6f, 0.6f, 0.6f }, lit ? 1.15f : 0.3f);

				for (int c = c0; c < c1; c++) {
					const float cx = -L * 0.5f + (c + 0.5f) * cell;
					auto FB = [&](float xx, float yy, float dep, float hx, float hy, float hz, V3 col) { addBoxShade(g_bldInter, IW(xx, y0 + yy, dep), a, U, inward, { hx, hy, hz }, col, lum); };
					const V3 chairC = { rf(0.15f, 0.55f), rf(0.15f, 0.45f), rf(0.20f, 0.60f) };
					if (type == 0) {
						FB(cx, 0.37f, 1.55f, 0.72f, 0.37f, 0.38f, { 0.74f, 0.70f, 0.62f });
						FB(cx, 0.93f, 1.80f, 0.27f, 0.19f, 0.03f, { 0.08f, 0.09f, 0.11f });
						FB(cx + 0.35f, 0.80f, 1.55f, 0.18f, 0.04f, 0.14f, { 0.85f, 0.85f, 0.85f });
						FB(cx, 0.46f, 0.90f, 0.23f, 0.04f, 0.23f, chairC);
						FB(cx, 0.78f, 0.66f, 0.23f, 0.28f, 0.04f, chairC * 0.9f);
						if (((c - c0) & 1) == 1) FB(cx - cell * 0.5f, 0.62f, 1.40f, 0.03f, 0.62f, 1.05f, { 0.62f, 0.66f, 0.70f });
						if (((c - c0) & 1) == 0) FB(cx, 0.68f, Dp - 0.38f, 0.45f, 0.68f, 0.30f, { 0.58f, 0.60f, 0.62f });
						if (c == c1 - 1) { FB(cx + cell * 0.35f, 0.25f, Dp - 0.5f, 0.22f, 0.25f, 0.22f, { 0.50f, 0.30f, 0.20f }); FB(cx + cell * 0.35f, 0.75f, Dp - 0.5f, 0.28f, 0.28f, 0.28f, { 0.15f, 0.48f, 0.20f }); }
					}
					else if (type == 1) {
						const V3 cur = { rf(0.55f, 0.95f), rf(0.45f, 0.85f), rf(0.45f, 0.85f) };
						FB(cx - ww * 0.5f - 0.05f, (wy0[f] - f * FH + wy1[f] - f * FH) * 0.5f + 0.15f, 0.12f, 0.20f, (wy1[f] - wy0[f]) * 0.5f + 0.3f, 0.05f, cur);
						FB(cx + ww * 0.5f + 0.05f, (wy0[f] - f * FH + wy1[f] - f * FH) * 0.5f + 0.15f, 0.12f, 0.20f, (wy1[f] - wy0[f]) * 0.5f + 0.3f, 0.05f, cur);
						if (((c - c0) & 1) == 0) {
							const V3 blank = { rf(0.35f, 0.85f), rf(0.35f, 0.75f), rf(0.45f, 0.90f) };
							FB(cx, 0.28f, 1.9f, 0.72f, 0.22f, 1.0f, { 0.88f, 0.86f, 0.80f });
							FB(cx, 0.50f, 2.25f, 0.70f, 0.06f, 0.70f, blank);
							FB(cx - 0.3f, 0.58f, 1.1f, 0.26f, 0.07f, 0.15f, { 0.97f, 0.97f, 0.97f }); FB(cx + 0.3f, 0.58f, 1.1f, 0.26f, 0.07f, 0.15f, { 0.97f, 0.97f, 0.97f });
							FB(cx + 1.0f, 0.30f, 1.2f, 0.20f, 0.30f, 0.20f, { 0.50f, 0.36f, 0.22f });
						}
						else {
							FB(cx, 0.26f, Dp - 0.75f, 0.85f, 0.26f, 0.40f, { rf(0.25f, 0.55f), rf(0.30f, 0.50f), rf(0.35f, 0.60f) });
							FB(cx, 0.65f, Dp - 0.30f, 0.85f, 0.30f, 0.10f, { 0.30f, 0.32f, 0.38f });
							FB(cx, 0.20f, Dp - 2.0f, 0.50f, 0.20f, 0.32f, { 0.55f, 0.40f, 0.26f });
							FB(cx + 1.1f, 0.55f, Dp - 0.2f, 0.45f, 0.30f, 0.05f, { 0.04f, 0.04f, 0.05f });
						}
					}
					else {
						if (((c - c0) & 1) == 0) { FB(cx, 0.40f, 1.1f, 0.55f, 0.40f, 0.55f, { 0.60f, 0.45f, 0.30f }); FB(cx, 0.85f, 1.1f, 0.18f, 0.05f, 0.18f, { 0.85f, 0.25f, 0.20f }); }
						FB(cx, 1.0f, Dp - 0.30f, cell * 0.5f - 0.12f, 1.0f, 0.25f, { 0.62f, 0.64f, 0.66f });
						for (int k = 0; k < 3; k++) FB(cx, 0.55f + k * 0.55f, Dp - 0.60f, cell * 0.5f - 0.2f, 0.10f, 0.04f, { rf(0.2f, 0.95f), rf(0.2f, 0.9f), rf(0.2f, 0.9f) });
						if (c == c0) FB(xm, 0.52f, Dp * 0.55f, std::min(1.6f, hxz * 0.6f), 0.52f, 0.40f, { 0.45f, 0.32f, 0.22f });
					}
				}
				c0 = c1;
			}
		}
	}
}

static Verts buildScenery() {
	std::random_device rd; std::mt19937 rng(rd());
	auto rf = [&](float a, float b) { return a + (b - a) * (float)(rng() / 4294967296.0); };
	sceneryOrigin();
	for (int i = 0; i < 4; i++) { g_nOff[i] = rf(0, 1000); g_rvPh[i] = rf(0, 2 * PI); }


	g_bumps.clear();
	auto addBump = [&](float rc, float ang, float h, float rad) { g_bumps.push_back({ g_cx + rc * cosf(ang), g_cz + rc * sinf(ang), h, rad }); };
	const int NM = 16;
	for (int i = 0; i < NM; i++) addBump(rf(1650, 1950), 2 * PI * (i + rf(-0.35f, 0.35f)) / NM, rf(140, 300), rf(260, 400));
	for (int i = 0; i < 14; i++) addBump(rf(800, 1250), rf(0, 2 * PI), rf(10, 40), rf(100, 220));
	for (int i = 0; i < 6; i++)  addBump(rf(280, 480), rf(0, 2 * PI), rf(8, 25), rf(80, 140));
	for (int i = 0; i < 6; i++)  addBump(rf(850, 1150), rf(0, 2 * PI), -rf(12, 24), rf(160, 300));

	houseInit();
	Verts v; g_curE = 0.0f;


	const int N = TERR_N; const float C = TERR_CELL, X0 = g_cx - N * C * 0.5f, Z0 = g_cz - N * C * 0.5f;
	std::vector<float> H((N + 1) * (N + 1));
	for (int iz = 0; iz <= N; iz++) for (int ix = 0; ix <= N; ix++) H[iz * (N + 1) + ix] = terrainH(X0 + ix * C, Z0 + iz * C);
	auto hAt = [&](int ix, int iz) { ix = std::min(N, std::max(0, ix)); iz = std::min(N, std::max(0, iz)); return H[iz * (N + 1) + ix]; };
	auto terrC = [&](float x, float z, float h, float ny) {
		float nz = vnoise(x / 35.0f + g_nOff[0], z / 35.0f + g_nOff[1]);
		V3 grass = { 0.30f + 0.08f * nz, 0.50f + 0.08f * nz, 0.24f + 0.04f * nz };
		V3 c = grass + (V3{ 0.18f, 0.38f, 0.19f } - grass) * (forestMask(x, z) * 0.6f);
		float sand = sm01((WATER_Y + 2.5f - h) / 2.5f);
		c = c + (V3{ 0.62f, 0.58f, 0.42f } - c) * sand;
		float rock = std::max(sm01((h - 70.0f) / 60.0f), sm01((0.82f - ny) / 0.2f));
		c = c + (V3{ 0.45f, 0.42f, 0.40f } - c) * rock;
		c = c + (V3{ 0.95f, 0.96f, 0.98f } - c) * sm01((h - 190.0f) / 50.0f);
		return c;
		};
	auto vtx = [&](int ix, int iz) {
		float x = X0 + ix * C, z = Z0 + iz * C, h = hAt(ix, iz);
		V3 n = norm({ -(hAt(ix + 1, iz) - hAt(ix - 1, iz)) / (2 * C), 1.0f, -(hAt(ix, iz + 1) - hAt(ix, iz - 1)) / (2 * C) });
		return Vtx{ { x, h, z }, n, terrC(x, z, h, n.y), 0.0f };
		};
	v.reserve((size_t)N * N * 6 + 400000);
	for (int iz = 0; iz < N; iz++) for (int ix = 0; ix < N; ix++) {
		Vtx a = vtx(ix, iz), b = vtx(ix + 1, iz), c = vtx(ix + 1, iz + 1), d = vtx(ix, iz + 1);
		v.push_back(a); v.push_back(b); v.push_back(c);
		v.push_back(a); v.push_back(c); v.push_back(d);
	}


	auto waterQuad = [&](V3 a, V3 b, V3 c, V3 d, V3 col) {
		addTri(v, a, b, c, col, a + V3{ 0, -1, 0 }); addTri(v, a, c, d, col, a + V3{ 0, -1, 0 });
		};
	const int NR = 240;
	for (int i = 0; i < NR; i++) {
		float a0 = 2 * PI * i / NR, a1 = 2 * PI * (i + 1) / NR;
		float r0 = riverR(a0), r1 = riverR(a1), w0 = riverW(a0) + 14.0f, w1 = riverW(a1) + 14.0f;
		V3 col = (i & 1) ? V3{ 0.10f, 0.30f, 0.52f } : V3{ 0.11f, 0.32f, 0.54f };
		waterQuad({ g_cx + (r0 - w0) * cosf(a0), WATER_Y, g_cz + (r0 - w0) * sinf(a0) }, { g_cx + (r0 + w0) * cosf(a0), WATER_Y, g_cz + (r0 + w0) * sinf(a0) },
			{ g_cx + (r1 + w1) * cosf(a1), WATER_Y, g_cz + (r1 + w1) * sinf(a1) }, { g_cx + (r1 - w1) * cosf(a1), WATER_Y, g_cz + (r1 - w1) * sinf(a1) }, col);
	}
	for (int i = 0; i < 48; i++) {
		float a0 = 2 * PI * i / 48, a1 = 2 * PI * (i + 1) / 48, rl = LAKE_R + 12.0f;
		addTri(v, { g_cx, WATER_Y, g_cz }, { g_cx + rl * cosf(a0), WATER_Y, g_cz + rl * sinf(a0) }, { g_cx + rl * cosf(a1), WATER_Y, g_cz + rl * sinf(a1) },
			{ 0.10f, 0.30f, 0.52f }, { g_cx, WATER_Y - 1.0f, g_cz });
	}


	auto landOK = [&](float x, float z, float minAdr, float maxSlope, float hmax, float& h) {
		float r, s; polar(x, z, r, s);
		if (r > 2050.0f || fabsf(r - g_R) < minAdr || r < LAKE_R + 22.0f) return false;
		float phi = atan2f(z - g_cz, x - g_cx);
		if (fabsf(r - riverR(phi)) < riverW(phi) + 24.0f) return false;
		h = terrainH(x, z);
		if (h < WATER_Y + 2.5f || h > hmax) return false;
		float sx = terrainH(x + 4, z) - terrainH(x - 4, z), sz = terrainH(x, z + 4) - terrainH(x, z - 4);
		return sqrtf(sx * sx + sz * sz) / 8.0f <= maxSlope;
		};


	std::vector<V3> housePos; std::vector<float> bldR;
	g_bldInter.clear(); g_bldGlass.clear(); g_colExtra.clear();
	const V3 walls[6] = { { 0.86f, 0.84f, 0.78f }, { 0.78f, 0.72f, 0.62f }, { 0.70f, 0.74f, 0.78f }, { 0.66f, 0.45f, 0.38f }, { 0.82f, 0.82f, 0.80f }, { 0.60f, 0.64f, 0.60f } };
	const V3 bands[4] = { { 0.46f, 0.47f, 0.50f }, { 0.74f, 0.74f, 0.72f }, { 0.34f, 0.38f, 0.44f }, { 0.58f, 0.55f, 0.50f } };
	auto tryBuilding = [&](float x, float z, float yaw, int minF, int maxF) {
		float h;
		if (!landOK(x, z, 48.0f, 0.12f, 70.0f, h)) return false;
		if ((x - g_hs.O.x) * (x - g_hs.O.x) + (z - g_hs.O.z) * (z - g_hs.O.z) < 45.0f * 45.0f) return false;
		{ float r, s; polar(x, z, r, s); if (fabsf(relS(s - 70.0f)) < 700.0f && fabsf(r - g_R) < 110.0f) return false; }
		const float w = rf(12.0f, 22.0f), d = rf(10.0f, 17.0f), rad = 0.5f * sqrtf(w * w + d * d) + 5.0f;
		for (size_t i = 0; i < housePos.size(); i++) { const V3& q = housePos[i]; if ((q.x - x) * (q.x - x) + (q.z - z) * (q.z - z) < (bldR[i] + rad) * (bldR[i] + rad)) return false; }
		float cyw = cosf(yaw), syw = sinf(yaw), hmx = -1e9f, hmn = 1e9f;
		for (int a = -1; a <= 1; a += 2) for (int b = -1; b <= 1; b += 2) {
			float hc = terrainH(x + cyw * (a * w * 0.5f) + syw * (b * d * 0.5f), z - syw * (a * w * 0.5f) + cyw * (b * d * 0.5f));
			hmx = std::max(hmx, hc); hmn = std::min(hmn, hc);
		}
		if (hmx - hmn > 3.0f) return false;
		const int nF = minF + (int)(rng() % (unsigned)(maxF - minF + 1));
		addBuilding(v, { x, hmx, z }, yaw, w, d, nF, walls[rng() % 6], bands[rng() % 4], hmx - hmn, rng);
		housePos.push_back({ x, hmx, z }); bldR.push_back(rad);
		return true;
		};
	for (const auto& stn : g_stations) {
		int placed = 0;
		for (int k = 0; k < 200 && placed < 7; k++) {
			Pose P = trackAt(stn.stopS + rf(-170.0f, 170.0f)); V3 rv = rightOf(P.f);
			V3 q = P.p + rv * (rf(50.0f, 150.0f) * ((rng() & 1) ? 1.0f : -1.0f));
			if (tryBuilding(q.x, q.z, atan2f(P.f.x, P.f.z) + (((rng() & 3) == 0) ? PI * 0.5f : 0.0f), 5, 11)) placed++;
		}
	}
	for (int vi = 0; vi < 8; vi++) {
		float rc = rf(150.0f, 1900.0f), ang = rf(0, 2 * PI), yawv = rf(0, 2 * PI);
		float vx = g_cx + rc * cosf(ang), vz = g_cz + rc * sinf(ang);
		int placed = 0;
		for (int k = 0; k < 160 && placed < 4; k++)
			if (tryBuilding(vx + rf(-110, 110), vz + rf(-110, 110), yawv + (((rng() & 3) == 0) ? PI * 0.5f : 0.0f), 3, 8)) placed++;
	}


	int trees = 0;
	for (int k = 0; k < 40000 && trees < 3200; k++) {
		float rr_ = sqrtf(rf(0, 1)) * 2000.0f, ang = rf(0, 2 * PI);
		float x = g_cx + rr_ * cosf(ang), z = g_cz + rr_ * sinf(ang), h;
		if (!landOK(x, z, 40.0f, 0.55f, 130.0f, h)) continue;
		if ((x - g_hs.O.x) * (x - g_hs.O.x) + (z - g_hs.O.z) * (z - g_hs.O.z) < 24.0f * 24.0f) continue;
		float prob = (0.05f + 0.95f * forestMask(x, z)) * (1.0f - sm01((h - 60.0f) / 70.0f));
		if (rf(0, 1) > prob) continue;
		bool tooClose = false;
		for (size_t bi = 0; bi < housePos.size(); bi++) { const V3& q = housePos[bi]; if ((q.x - x) * (q.x - x) + (q.z - z) * (q.z - z) < (bldR[bi] + 3.0f) * (bldR[bi] + 3.0f)) { tooClose = true; break; } }
		if (tooClose) continue;
		float sc = rf(0.8f, 1.5f), yaw = rf(0, 2 * PI);
		V3 p = { x, h - 0.3f, z };
		if (h > 30.0f || (rng() & 3) < 2) addConifer(v, p, yaw, sc, { rf(0.10f, 0.16f), rf(0.30f, 0.40f), rf(0.14f, 0.20f) });
		else addBroadleaf(v, p, yaw, sc, ((rng() % 10) == 0) ? V3{ 0.70f, 0.40f, 0.10f } : V3{ rf(0.18f, 0.28f), rf(0.42f, 0.52f), rf(0.14f, 0.22f) });
		trees++;
	}

	buildHouseGeometry(g_hvx);
	v.insert(v.end(), g_hvx.stat.begin(), g_hvx.stat.end());
	return v;
}






static const char* kShader = R"HLSL(#ifndef USE_DXR
#define USE_DXR 0
#endif
cbuffer DrawCB : register(b0) { row_major float4x4 gWorld; float4 gDraw; };
cbuffer FrameCB : register(b1) {
	row_major float4x4 vp;
	row_major float4x4 invVp;
	row_major float4x4 prevVp;
	float4 camPos;
	float4 sunDir;
	float4 env;
	float4 sp0[4]; float4 sp1[4]; float4 spd[4]; float4 spp;   // headlights per train (spd[h].w = 1 when active)
	float4 rtA;
	float4 rtB;
	float4 rtC;
	float4 rtD;
	float4 rtE;
	float4 rtF;
	float4 rtG;
	float4 rtH;
	uint4 roots;
	uint4 triBase;
	uint4 carType;
	row_major float4x4 carW[4];
	row_major float4x4 carInv[4];
	float4 lampP[16];   // same as LAMP_HW_MAX (C++ side)
	float4 lampI;
	float4 flA;
	float4 flB;
	float4 flC;
	float4 hlO;
	float4 hlRot;
	float4 hlP[4];
	float4 hlC[4];
	float4 hlMn[4];
	float4 hlMx[4];
};
StructuredBuffer<float4> gNodes : register(t0);
StructuredBuffer<float4> gTris : register(t1);
#if USE_DXR
RaytracingAccelerationStructure gTLAS : register(t2);
#endif
Texture2D<float4> gColor : register(t3);
Texture2DMS<float> gDepth : register(t4);
Texture2D<float4> gAlb : register(t6);
SamplerState gSamp : register(s0);

struct VI { float3 p : POSITION; float3 n : NORMAL; float3 c : COLOR; float e : TEXCOORD0; };
struct VO { float4 p : SV_POSITION; float3 n : NORMAL; float3 c : COLOR; float e : TEXCOORD0; float3 wp : TEXCOORD1; };
VO vs(VI i) {
	VO o;
	float4 w = mul(float4(i.p, 1), gWorld);
	o.p = mul(w, vp);
	o.n = mul(i.n, (float3x3)gWorld);
	o.c = i.c;
	o.e = i.e;
	o.wp = w.xyz;
	return o;
}

struct RHit { float t; float3 n; float3 col; float emis; };
float3 unpackCol(uint p) { return float3(p & 255u, (p >> 8) & 255u, (p >> 16) & 255u) * (1.0 / 255.0); }
float3 sfix(float3 d) {
	return float3(abs(d.x) < 1e-8 ? 1e-8 : d.x, abs(d.y) < 1e-8 ? 1e-8 : d.y, abs(d.z) < 1e-8 ? 1e-8 : d.z);
}

#if !USE_DXR

bool hitTri(uint ti, float3 o, float3 d, float tmax, out float t)
{
	float4 a = gTris[ti * 3];
	float3 p0 = a.xyz;
	float3 e1 = gTris[ti * 3 + 1].xyz - p0;
	float3 e2 = gTris[ti * 3 + 2].xyz - p0;
	float3 pv = cross(d, e2);
	float det = dot(e1, pv);
	t = 0;
	if (abs(det) < 1e-12) return false;
	float idet = 1.0 / det;
	float3 tv = o - p0;
	float u = dot(tv, pv) * idet;
	float3 qv = cross(tv, e1);
	float v = dot(d, qv) * idet;
	t = dot(e2, qv) * idet;
	return (u >= 0.0 && v >= 0.0 && (u + v) <= 1.0 && t > 1e-4 && t < tmax);
}

bool bvhTrace(uint root, float3 o, float3 d, inout float tmax, bool anyHit, out uint hitIdx)
{
	hitIdx = 0;
	bool found = false;
	float3 inv = 1.0 / sfix(d);
	uint stack[64];
	int sp = 0;
	uint node = root;
	[loop] while (true)
	{
		float4 a = gNodes[node * 2];
		float4 b = gNodes[node * 2 + 1];
		float3 t0 = (a.xyz - o) * inv;
		float3 t1 = (b.xyz - o) * inv;
		float3 tn = min(t0, t1);
		float3 tf = max(t0, t1);
		float tmin = max(max(tn.x, tn.y), max(tn.z, 0.0));
		float tmx = min(min(tf.x, tf.y), min(tf.z, tmax));
		bool popNode = true;
		if (tmin <= tmx)
		{
			uint cnt = asuint(b.w);
			uint aa = asuint(a.w);
			if (cnt > 0)
			{
				[loop] for (uint k = 0; k < cnt; k++)
				{
					float t;
					if (hitTri(aa + k, o, d, tmax, t))
					{
						tmax = t; hitIdx = aa + k; found = true;
						if (anyHit) k = cnt;
					}
				}
			}
			else
			{
				stack[sp] = aa;
				sp++;
				node = node + 1;
				popNode = false;
			}
		}
		if (anyHit && found) break;
		if (popNode)
		{
			if (sp == 0) break;
			sp--;
			node = stack[sp];
		}
	}
	return found;
}

struct SceneHit { float t; uint tri; int car; };

bool traceSW(float3 o, float3 d, float tmax, bool anyHit, out SceneHit h)
{
	h.t = tmax; h.tri = 0; h.car = -1;
	bool hit = false;
	uint tri = 0;
	float tm = tmax;
	if (bvhTrace(roots.x, o, d, tm, anyHit, tri)) { hit = true; h.t = tm; h.tri = tri; h.car = -1; }
	[loop] for (int c = 0; c < 4; c++)
	{
		if (!(anyHit && hit))
		{
			float3 ol = mul(float4(o, 1), carInv[c]).xyz;
			float3 dl = mul(d, (float3x3)carInv[c]);
			tm = h.t;
			if (bvhTrace(roots[carType[c]], ol, dl, tm, anyHit, tri)) { hit = true; h.t = tm; h.tri = tri; h.car = c; }
		}
	}
	return hit;
}

RHit resolveHit(SceneHit h)
{
	RHit r;
	float4 a = gTris[h.tri * 3];
	float3 p0 = a.xyz;
	float3 e1 = gTris[h.tri * 3 + 1].xyz - p0;
	float3 e2 = gTris[h.tri * 3 + 2].xyz - p0;
	float3 n = normalize(cross(e1, e2));
	if (h.car >= 0) n = normalize(mul(n, (float3x3)carInv[h.car]));
	uint pk = asuint(a.w);
	r.t = h.t; r.n = n; r.col = unpackCol(pk);
	r.emis = (((pk >> 24) & 255u) > 127u) ? 1.0 : 0.0;
	return r;
}

bool traceRay(float3 o, float3 d, float tmax, bool anyHit, out RHit r)
{
	r.t = tmax; r.n = float3(0, 1, 0); r.col = float3(0, 0, 0); r.emis = 0;
	SceneHit h;
	bool b = traceSW(o, d, tmax, anyHit, h);
	if (b) { r.t = h.t; if (!anyHit) r = resolveHit(h); }
	return b;
}
#else

bool traceRay(float3 o, float3 d, float tmax, bool anyHit, out RHit r)
{
	r.t = tmax; r.n = float3(0, 1, 0); r.col = float3(0, 0, 0); r.emis = 0;
	RayDesc ray;
	ray.Origin = o; ray.Direction = d; ray.TMin = 0.0; ray.TMax = tmax;
	RayQuery<RAY_FLAG_FORCE_OPAQUE> q;
	uint fl = anyHit ? RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH : RAY_FLAG_NONE;
	q.TraceRayInline(gTLAS, fl, 0xFF, ray);
	while (q.Proceed()) { }
	bool b = (q.CommittedStatus() == COMMITTED_TRIANGLE_HIT);
	if (b)
	{
		r.t = q.CommittedRayT();
		if (!anyHit)
		{
			uint inst = q.CommittedInstanceID();
			uint ti = triBase[inst] + q.CommittedPrimitiveIndex();
			float4 a = gTris[ti * 3];
			float3 p0 = a.xyz;
			float3 e1 = gTris[ti * 3 + 1].xyz - p0;
			float3 e2 = gTris[ti * 3 + 2].xyz - p0;
			float3 n = normalize(cross(e1, e2));
			if (inst != 0)
			{
				float3x4 w2o = q.CommittedWorldToObject3x4();
				n = normalize(mul(n, (float3x3)w2o));
			}
			uint pk = asuint(a.w);
			r.n = n; r.col = unpackCol(pk);
			r.emis = (((pk >> 24) & 255u) > 127u) ? 1.0 : 0.0;
		}
	}
	return b;
}
#endif
float sqr(float x) { return x * x; }
float ign(float2 p) { return frac(52.9829189 * frac(dot(p, float2(0.06711056, 0.00583715)))); }

void basisOf(float3 n, out float3 t, out float3 b)
{
	float3 u = (abs(n.y) < 0.99) ? float3(0, 1, 0) : float3(1, 0, 0);
	t = normalize(cross(u, n));
	b = cross(n, t);
}
float3 cosHemi(float3 n, float u1, float u2)
{
	float3 t, b;
	basisOf(n, t, b);
	float r = sqrt(u1), ph = 6.2831853 * u2;
	return normalize(t * (r * cos(ph)) + b * (r * sin(ph)) + n * sqrt(max(0.0, 1.0 - u1)));
}

float hash12(float2 p)
{
	float3 p3 = frac(float3(p.x, p.y, p.x) * 0.1031);
	p3 += dot(p3, p3.yzx + 33.33);
	return frac((p3.x + p3.y) * p3.z);
}
float vn2(float2 p)
{
	float2 i = floor(p), f = frac(p);
	f = f * f * (3.0 - 2.0 * f);
	float a = hash12(i), b = hash12(i + float2(1, 0)), c = hash12(i + float2(0, 1)), d = hash12(i + float2(1, 1));
	return lerp(lerp(a, b, f.x), lerp(c, d, f.x), f.y);
}
float fbm2(float2 p, int oct)
{
	float s = 0.0, a = 0.5, w = 0.0;
	[loop] for (int k = 0; k < oct; k++)
	{
		s += a * vn2(p); w += a;
		p = p * 2.03 + float2(17.1, 9.2);
		a *= 0.5;
	}
	return s / w;
}

float3 shoulder(float3 c)
{
	float3 t = max(c - 0.78, 0.0);
	return min(c, 0.78) + 0.22 * (1.0 - exp(-t / 0.22));
}


float sunLowK() { return 1.0 - smoothstep(0.02, 0.45, sunDir.y); }
float3 skyBase(float3 d)
{
	float y = saturate(d.y);
	float low = sunLowK();
	float3 hor = float3(0.72, 0.85, 0.98);
	float3 zen = float3(0.12, 0.34, 0.88);
	float az = saturate(dot(normalize(d.xz + float2(1e-5, 0.0)), normalize(sunDir.xz + float2(1e-5, 0.0))) * 0.5 + 0.5);
	hor = lerp(hor, lerp(float3(0.85, 0.62, 0.55), float3(1.0, 0.50, 0.22), az * az), low);
	zen = lerp(zen, float3(0.16, 0.20, 0.46), low * 0.7);
	float3 c = lerp(hor, zen, pow(y, 0.55));
	float sd = saturate(dot(d, sunDir.xyz));
	float3 sc = lerp(float3(1.0, 0.82, 0.55), float3(1.0, 0.55, 0.25), low);
	c += sc * (pow(sd, 6.0) * 0.18 + pow(sd, 64.0) * 0.25);
	return c;
}
float3 skyCol(float3 d)
{
	float nf = sunDir.w;
	float3 night = float3(0.01, 0.015, 0.045);
	if (nf > 0.999) return night;
	float sd = saturate(dot(d, sunDir.xyz));
	float3 sc = lerp(float3(1.0, 0.85, 0.6), float3(1.0, 0.6, 0.3), sunLowK());
	float3 day = skyBase(d) + sc * pow(sd, 512.0) * 4.0;
	return lerp(day, night, nf);
}

float3 fogCol(float3 d)
{
	float3 day = skyBase(normalize(float3(d.x, max(d.y, 0.0) * 0.3, d.z)));
	return lerp(day, float3(0.01, 0.015, 0.045), sunDir.w);
}


float3 surfDetail(float3 wp, float3 n, float3 ng, float dist, float3 c, float amt)
{
	float2 uv = (abs(n.y) > 0.6) ? wp.xz : ((abs(n.x) > abs(n.z)) ? wp.zy : wp.xy);
	float v = vn2(uv * 0.35) - 0.5;
	float w2 = saturate(1.0 - dist / 160.0), w3 = saturate(1.0 - dist / 40.0);
	if (rtG.z > 1.5) v += (vn2(uv * 1.7 + 5.3) - 0.5) * 0.7 * w2;
	float g = 0.0;
	if (rtG.z > 2.5) g = ((vn2(uv * 7.0 + 11.7) - 0.5) * 0.9 + (vn2(uv * 19.0 + 2.3) - 0.5) * 0.4) * w3;
	float grass = (c.g > c.r * 1.25 && c.g > c.b * 1.4) ? 1.0 : 0.0;
	float k = 1.0 + v * (0.16 + 0.08 * grass) + g * (0.10 + 0.06 * grass);
	float3 a = c * float3(k, k * (1.0 + 0.05 * grass * v), k);
	if (grass > 0.5 && rtG.z > 1.5 && ng.y > 0.995 && n.y > 0.995 && c.g > 0.42)
	{

		float dd = saturate(1.0 - dist / 45.0);
		float2 fc = floor(uv * 2.5);
		float fh = hash12(fc + 3.7);
		if (fh > 0.975 && dd > 0.0)
		{
			float m = smoothstep(0.16, 0.05, length(frac(uv * 2.5) - 0.5)) * dd;
			float3 fcol = (fh > 0.99) ? float3(1.0, 0.95, 0.60) : ((fh > 0.983) ? float3(0.95, 0.95, 1.0) : float3(0.95, 0.55, 0.75));
			a = lerp(a, fcol, m * 0.85);
		}
	}
	if (grass < 0.5 && rtG.z > 2.5)
	{

		float wp2 = saturate(1.0 - dist / 28.0);
		a *= 1.0 + (vn2(uv * 13.0 + 7.7) - 0.5) * 0.18 * wp2 * step(0.9, ng.y);
	}
	return lerp(c, a, amt);
}

)HLSL"
R"HLSL(
float waveH(float2 p, float t)
{
	return vn2(p * 0.30 + float2(t * 0.20, t * 0.13)) + 0.5 * vn2(p * 0.85 + float2(-t * 0.31, t * 0.07)) + 0.25 * vn2(p * 2.3 + float2(t * 0.5, -t * 0.4));
}
float3 waterN(float2 p, float t, float dist)
{
	float e = 0.25;
	float h0 = waveH(p, t), hx = waveH(p + float2(e, 0.0), t), hz = waveH(p + float2(0.0, e), t);
	float k = (0.25 * saturate(1.0 - dist / 600.0) + 0.03) / e;
	return normalize(float3(-(hx - h0) * k, 1.0, -(hz - h0) * k));
}


float3 skyFull(float3 d)
{
	float nf = sunDir.w;
	float3 c = skyCol(d);
	if (nf > 0.01)
	{
		if (d.y > 0.0)
		{
			float2 sp = d.xz / (d.y + 0.35) * 55.0;
			float2 cell = floor(sp);
			float hh = hash12(cell);
			float2 f = frac(sp) - 0.5 - (float2(hash12(cell + 7.1), hash12(cell + 3.3)) - 0.5) * 0.6;
			float star = step(0.985, hh) * smoothstep(0.16, 0.0, length(f)) * (0.35 + 0.65 * (hh - 0.985) / 0.015);
			c += float3(0.85, 0.90, 1.0) * star * saturate(d.y * 4.0) * nf * nf;
		}
		float md = dot(d, -sunDir.xyz);
		c += float3(0.85, 0.90, 1.0) * (smoothstep(0.99955, 0.99985, md) * 1.6 + pow(saturate(md), 300.0) * 0.10) * nf * nf;
	}
	if (nf < 0.999 && rtG.y > 0.5 && d.y > 0.012)
	{
		float t = rtF.x;
		float fadeH = smoothstep(0.012, 0.16, d.y) * (1.0 - nf);
		float k = smoothstep(0.0, 0.35, sunDir.y);
		float3 lit = lerp(float3(1.0, 0.62, 0.42), float3(1.0, 0.98, 0.94), k) * lerp(0.7, 1.0, k);
		float3 dark = lerp(float3(0.42, 0.34, 0.45), float3(0.62, 0.72, 0.88), k);
		if (rtG.y > 1.5)
		{

			float2 qb = (camPos.xz + d.xz * (2600.0 / d.y) + float2(t * 9.0, t * 4.0)) * 0.00085;
			float db = fbm2(qb, 4);
			float covb = smoothstep(0.47, 0.66, db) * 0.80;
			float shb = saturate((db - fbm2(qb + sunDir.xz / max(sunDir.y, 0.2) * 0.05, 2)) * 3.0 + 0.6);
			c = lerp(c, lerp(float3(0.78, 0.84, 0.95) * lerp(0.7, 1.0, k), lit, shb), covb * fadeH);
		}

		float2 q = (camPos.xz + d.xz * (1200.0 / d.y) + float2(t * 6.0, t * 2.5)) * 0.00042;
		float dn = fbm2(q, (rtG.y > 1.5) ? 5 : 3);
		float cov = smoothstep(0.54, 0.76, dn);
		if (cov > 0.001)
		{
			float dn2 = fbm2(q + sunDir.xz / max(sunDir.y, 0.2) * 0.05, (rtG.y > 1.5) ? 4 : 2);
			float shade = saturate((dn - dn2) * 3.2 + 0.58);
			c = lerp(c, lerp(dark, lit, shade), cov * 0.95 * fadeH);
		}
	}
	return c;
}

bool occluded(float3 o, float3 d, float tmax)
{
	RHit h;
	return traceRay(o, d, tmax, true, h);
}


float skyLevel() { return smoothstep(-0.12, 0.30, sunDir.y); }
float3 hemiAmb(float3 n)
{
	float3 a = lerp(float3(0.37, 0.34, 0.27), float3(0.46, 0.56, 0.74), saturate(n.y * 0.5 + 0.5));
	float k = skyLevel();
	return a * lerp(float3(0.70, 0.55, 0.62), float3(1.0, 1.0, 1.0), k) * lerp(0.60, 1.0, k);
}
float3 sunCol() { return lerp(float3(1.0, 0.45, 0.18), float3(1.0, 0.95, 0.86), smoothstep(0.0, 0.30, sunDir.y)); }
float sunI() { return 0.80 * smoothstep(0.0, 0.12, sunDir.y); }

float3 shadeHit(RHit h, float3 rd, float3 hp, bool withShadow)
{
	float3 n = h.n;
	if (dot(n, rd) > 0.0) n = -n;
	float nf = sunDir.w;
	float3 night = h.col * ((h.emis > 0.5) ? 0.8 : 0.05);
	if (nf > 0.999) return night;
	float ndl = saturate(dot(n, sunDir.xyz));
	float vis = 1.0;
	if (withShadow && ndl > 0.0 && sunDir.y > 0.01) vis = occluded(hp + n * 0.05, sunDir.xyz, 2000.0) ? 0.0 : 1.0;
	float3 day;
	if (h.emis > 0.5) day = h.col * (0.80 + 0.20 * saturate(n.y));
	else day = h.col * (hemiAmb(n) + sunI() * ndl * vis * sunCol());
	return lerp(day, night, nf);
}


float3 shadeHit2(RHit h, float3 rd, float3 hp, float u1, float u2)
{
	float3 n = h.n;
	if (dot(n, rd) > 0.0) n = -n;
	if (sunDir.w > 0.5 || h.emis > 0.5) return shadeHit(h, rd, hp, true);
	float ndl = saturate(dot(n, sunDir.xyz));
	float vis = 1.0;
	if (ndl > 0.0 && sunDir.y > 0.01) vis = occluded(hp + n * 0.05, sunDir.xyz, 2000.0) ? 0.0 : 1.0;
	float3 d2 = cosHemi(n, u1, u2);
	float3 o2 = hp + n * 0.05;
	RHit h2;
	float3 amb2;
	if (traceRay(o2, d2, 120.0, false, h2)) amb2 = shadeHit(h2, d2, o2 + d2 * h2.t, false) * 0.75;
	else amb2 = skyCol(d2) * 0.8;
	return h.col * (amb2 + sunI() * ndl * vis * sunCol());
}

)HLSL"
R"HLSL(
float spotL(float3 sp, float3 wp, float3 n, float3 dir)
{
	float3 L = sp - wp;
	float dist = max(length(L), 0.001);
	L /= dist;
	float cd = dot(-L, dir);
	float cone = saturate((cd - spp.y) / (spp.x - spp.y));
	cone = cone * cone * (3.0 - 2.0 * cone);
	float att = saturate(1.0 - dist / spp.z);
	att *= att;
	return cone * att * saturate(dot(n, L)) * spp.w;
}


float3 lampLight(float3 wp, float3 n)
{
	float3 acc = float3(0, 0, 0);
	[loop] for (int k = 0; k < 16; k++)
	{
		if (lampP[k].w > 0.5)
		{
			float3 L = lampP[k].xyz - wp;
			float d = max(length(L), 0.05);
			if (d < 22.0)
			{
				L /= d;
				float att = saturate(1.0 - d / 22.0);
				att *= att;
				float down = saturate(L.y * 1.25 + 0.12);
				float lam = saturate(dot(n, L) * 0.85 + 0.15);
				acc += att * down * lam;
			}
		}
	}
	return acc * lampI.x * float3(1.0, 0.82, 0.52);
}


float flashLight(float3 wp, float3 n)
{
	if (flA.w < 0.5) return 0.0;
	float3 L = flA.xyz - wp;
	float dist = max(length(L), 0.001);
	L /= dist;
	float cd = dot(-L, flB.xyz);
	float cone = saturate((cd - flB.w) / (flC.x - flB.w));
	cone = cone * cone * (3.0 - 2.0 * cone);
	float att = saturate(1.0 - dist / flC.y);
	att *= att;
	return cone * att * saturate(dot(n, L)) * flC.z;
}


float3 houseLocal(float3 wp)
{
	float3 d = wp - hlO.xyz;
	return float3(d.x * hlRot.x + d.z * hlRot.y, d.y, d.x * hlRot.z + d.z * hlRot.w);
}
float inRoom(float3 lq, float ex)
{
	float m = 0.0;
	[unroll] for (int k = 0; k < 4; k++)
	{
		float3 lo = hlMn[k].xyz - ex, hi = hlMx[k].xyz + ex;
		if (all(lq > lo) && all(lq < hi)) m = 1.0;
	}
	return m;
}
float3 houseLight(float3 wp, float3 lq, float3 n)
{
	float3 acc = float3(0.0, 0.0, 0.0);
	[unroll] for (int k = 2; k < 4; k++)
	{
		float3 c = hlC[k].xyz;
		if (c.x + c.y + c.z > 0.001)
		{
			float3 lo = hlMn[k].xyz - 0.04, hi = hlMx[k].xyz + 0.04;
			if (all(lq > lo) && all(lq < hi))
			{
				float3 L = hlP[k].xyz - wp;
				float d = max(length(L), 0.05);
				L /= d;
				float att = saturate(1.0 - sqr(d / hlP[k].w)) / (1.0 + 0.30 * d * d);
				float down = lerp(1.0, saturate(L.y * 1.25 + 0.1), hlC[k].w);
				float lam = saturate(dot(n, L) * 0.85 + 0.15);
				acc += c * (att * lam * down);
			}
		}
	}
	return acc;
}


float3 houseFill(float3 col, float3 alb, float3 n, float3 lq)
{
	[unroll] for (int k = 0; k < 2; k++)
	{
		float lv = hlMn[k].w;
		if (lv > 0.001)
		{
			float3 lo = hlMn[k].xyz - 0.04, hi = hlMx[k].xyz + 0.04;
			if (all(lq > lo) && all(lq < hi))
			{
				float3 tgt = alb * (0.80 + 0.20 * saturate(n.y)) * float3(1.0, 1.0, 0.96) * hlMx[k].w;
				col = lerp(col, tgt, saturate(lv));
			}
		}
	}
	return col;
}

float3 houseGlow(float3 lq)
{
	float3 acc = float3(0.0, 0.0, 0.0);
	[unroll] for (int k = 0; k < 2; k++)
	{
		float3 lo = hlMn[k].xyz - 0.16, hi = hlMx[k].xyz + 0.16;
		if (all(lq > lo) && all(lq < hi)) acc += hlC[k].xyz;
	}
	return acc;
}

)HLSL"
R"HLSL(struct PSOut { float4 c : SV_Target0; float4 a : SV_Target1; };
PSOut ps(VO i)
{
	float3 n = normalize(i.n);
	float3 cg = cross(ddx(i.wp), ddy(i.wp));
	float3 ng = (dot(cg, cg) > 1e-20) ? normalize(cg) : n;
	if (dot(ng, n) < 0.0) ng = -ng;
	float3 toCam = camPos.xyz - i.wp;
	float camDist = length(toCam);
	float3 V = toCam / max(camDist, 1e-4);
	float2 pix = floor(i.p.xy / rtD.x);
	float n0 = ign(pix);
	float n1 = ign(pix + float2(37.0, 17.0));
	float nightF = env.x;
	float dayF = 1.0 - nightF;
	bool dayOn = dayF > 0.004;
	bool nightOn = nightF > 0.004;
	float isTrain = gDraw.x;
	bool glass = gDraw.y > 0.5;
	bool interior = (i.e > 1.5);
	bool lampHead = (i.e > 1.1 && i.e < 1.4);
	bool rtOn = (camDist < rtB.z) && !interior;
	float3 P = i.wp + ng * (0.03 + 0.0008 * camDist);
	float3 col;
	float3 lq = float3(0.0, 0.0, 0.0);
	float rm = 0.0;
	if (hlO.w > 0.5) { lq = houseLocal(i.wp); rm = inRoom(lq, 0.04); }


	bool water = dayOn && (isTrain < 0.5) && (abs(i.wp.y + 6.0) < 0.03) && (n.y > 0.98) && (i.c.b > i.c.r * 1.8);
	float3 alb = i.c;
	if (rtG.z > 0.5 && isTrain < 0.5 && !water) alb = surfDetail(i.wp, n, ng, camDist, i.c, lerp(1.0, 0.3, rm));

	float3 colDay = float3(0.0, 0.0, 0.0);
	float3 colNight = float3(0.0, 0.0, 0.0);

	if (dayOn)
	{
		if (water)
		{
			float3 wn = waterN(i.wp.xz, rtF.x, camDist);
			float ndv = saturate(dot(wn, V));
			float fr = saturate(0.02 + 0.98 * pow(1.0 - ndv, 5.0) + 0.04);
			float3 R = reflect(-V, wn);
			R.y = abs(R.y);
			float3 rc = skyCol(R);
			if (rtG.w > 1.5 && rtOn)
			{
				RHit wh;
				float3 RO = i.wp + float3(0.0, 0.05, 0.0);
				if (traceRay(RO, R, 1500.0, false, wh)) rc = shadeHit(wh, R, RO + R * wh.t, false);
			}
			rc += float3(1.0, 0.9, 0.7) * pow(saturate(dot(R, sunDir.xyz)), 700.0) * 8.0 * saturate(sunDir.y * 6.0);
			colDay = lerp(i.c * 0.8, rc, fr);
		}
		else
		{
			float ndl = saturate(dot(n, sunDir.xyz));
			float vis = 1.0, ao = 1.0;
			float3 ambient = hemiAmb(n);
			if (rtOn && !glass)
			{
				if (ndl > 0.0 && rtA.x > 0.5 && sunDir.y > 0.01)
				{
					int ns = (int)rtA.x;
					float3 st, sb;
					basisOf(sunDir.xyz, st, sb);
					float acc = 0.0;
					[loop] for (int k = 0; k < ns; k++)
					{
						float u1 = frac(n0 + k * 0.7548776662);
						float u2 = frac(n1 + k * 0.5698402909);
						float rr = rtA.y * sqrt(u1), ph = 6.2831853 * u2;
						float3 dir = normalize(sunDir.xyz + st * (rr * cos(ph)) + sb * (rr * sin(ph)));
						float dnn = dot(dir, ng);
						if (dnn < 0.03) dir = normalize(dir + ng * (0.03 - dnn));
						acc += occluded(P, dir, 3000.0) ? 0.0 : 1.0;
					}
					vis = acc / ns;
				}
				if (rtA.z > 0.5)
				{
					int na = (int)rtA.z;
					float occ = 0.0;
					[loop] for (int k = 0; k < na; k++)
					{
						float u1 = frac(n1 + k * 0.7548776662 + 0.21);
						float u2 = frac(n0 + k * 0.5698402909 + 0.63);
						float3 dir = cosHemi(ng, u1, u2);
						occ += occluded(P, dir, rtA.w) ? 1.0 : 0.0;
					}

					float aoStr = (na < 2) ? 0.35 : 0.7;
					ao = 1.0 - aoStr * occ / na;
				}
				if (rtB.x > 0.5)
				{
					int ngi = (int)rtB.x;
					float3 acc = float3(0, 0, 0);
					[loop] for (int k = 0; k < ngi; k++)
					{
						float u1 = frac(n0 + k * 0.7548776662 + 0.47);
						float u2 = frac(n1 + k * 0.5698402909 + 0.11);
						float3 dir = cosHemi(ng, u1, u2);
						RHit h;
						if (traceRay(P, dir, 200.0, false, h))
						{
							float3 hp = P + dir * h.t;
							if (rtH.y > 0.5) acc += shadeHit2(h, dir, hp, frac(n1 + k * 0.618 + 0.30), frac(n0 + k * 0.7548776662 + 0.77));
							else acc += shadeHit(h, dir, hp, true);
						}
						else acc += skyCol(dir);
					}
					ambient = acc / ngi * 0.8;
				}
			}
			if (rtB.x < 0.5) ambient *= 1.0 - 0.45 * rm;
			colDay = alb * (ambient * ao + sunI() * ndl * vis * sunCol());

			if (isTrain > 0.5 && !glass && i.e < 0.5 && ndl > 0.0)
			{
				float3 Hh = normalize(sunDir.xyz + V);
				colDay += float3(1.0, 0.95, 0.85) * pow(saturate(dot(n, Hh)), 120.0) * 0.55 * vis * saturate(sunDir.y * 6.0);
			}
			if (rtG.z > 1.5 && isTrain < 0.5 && ng.y > 0.99 && i.c.g > i.c.r * 1.25 && camDist < 30.0)
			{
				float dw = step(0.985, hash12(floor(i.wp.xz * 30.0))) * saturate(1.0 - camDist / 30.0);
				colDay += float3(1.0, 1.0, 0.95) * dw * 0.45 * saturate(ndl + 0.2);
			}
		}
	}
	if (nightOn)
	{
		if (i.e > 0.5)
		{
			colNight = i.c * (0.80 + 0.20 * saturate(n.y)) * float3(1.0, 1.0, 0.96);
		}
		else
		{
			colNight = alb * (0.05 + 0.04 * saturate(n.y)) * float3(0.55, 0.65, 1.0);
			if (env.y > 0.01)
			{
				float sSum = 0.0;
				[loop] for (int h = 0; h < 4; h++)
				{
					if (spd[h].w < 0.5) continue;
					float3 hd = spd[h].xyz;
					float s0 = spotL(sp0[h].xyz, i.wp, n, hd);
					float s1 = spotL(sp1[h].xyz, i.wp, n, hd);
					if (rtD.y > 0.5 && rtOn)
					{
						if (s0 > 0.01) { float3 L = sp0[h].xyz - P; float dl = length(L); if (occluded(P, L / dl, dl - 0.4)) s0 = 0.0; }
						if (s1 > 0.01) { float3 L = sp1[h].xyz - P; float dl = length(L); if (occluded(P, L / dl, dl - 0.4)) s1 = 0.0; }
					}
					sSum += s0 + s1;
				}
				colNight += alb * sSum * float3(1.0, 0.95, 0.80) * env.y;
				colNight += alb * flashLight(i.wp, n) * float3(1.0, 0.96, 0.85);
			}
		}
	}
	col = lerp(colNight, colDay, dayF);

	if (interior) col = i.c * (0.80 + 0.20 * saturate(n.y)) * float3(1.0, 1.0, 0.96);


	if (hlO.w > 0.5 && !interior && !lampHead && isTrain < 0.5)
	{
		if (glass) col += houseGlow(lq);
		else { col = houseFill(col, alb, n, lq); col += alb * houseLight(i.wp, lq, n); }
	}


	float lampMul = (i.e > 0.5) ? dayF : 1.0;
	if (!interior && !glass && !water && !lampHead && lampI.x > 0.0)
		col += alb * lampLight(i.wp, n) * lampMul;
	if (lampHead) col = float3(1.0, 0.93, 0.72) * 2.4;


	if (rtB.y > 0.5 && rtOn && isTrain > 0.5 && (i.e < 0.5 || glass))
	{
		float3 rn = (dot(n, V) < 0.0) ? -n : n;
		float3 R = reflect(-V, rn);
		float ndv = saturate(dot(rn, V));
		float fr = pow(1.0 - ndv, 5.0);
		fr = glass ? (0.06 + 0.94 * fr) * 0.8 : (0.10 + 0.90 * fr) * 0.9;
		float3 RO = i.wp + rn * 0.03;
		float3 rc;
		RHit h;
		if (traceRay(RO, R, 600.0, false, h)) rc = shadeHit(h, R, RO + R * h.t, rtB.x > 0.5);
		else rc = skyCol(R);
		col = lerp(col, rc, fr);
	}


	if (rtH.x > 0.5 && rtOn && dayF > 0.5 && sunDir.y > 0.05 && !glass)
	{
		int nv = (int)rtH.x;
		float Lm = min(camDist, 350.0);
		float shf = 0.0;
		[loop] for (int k = 0; k < nv; k++)
		{
			float t = Lm * ((float)k + frac(n0 + k * 0.618)) / (float)nv;
			shf += occluded(camPos.xyz - V * t, sunDir.xyz, 800.0) ? 0.0 : 1.0;
		}
		shf /= (float)nv;
		float sdv = saturate(dot(-V, sunDir.xyz));
		float phase = 0.35 + 1.6 * pow(sdv, 6.0);
		col += float3(1.0, 0.90, 0.72) * shf * (1.0 - exp(-Lm * 0.0011)) * phase * 0.20;
	}


	if (!interior)
	{
		float fa = (1.0 - exp(-camDist * 0.00028)) * 0.92 * smoothstep(0.0, 0.6, dayF);
		col = lerp(col, fogCol(-V), fa);
	}
	col = shoulder(col);

	PSOut o;
	o.c = float4(col, (isTrain > 0.5) ? 0.0 : 1.0);
	o.a = float4(water ? float3(0.9, 0.9, 0.9) : alb, rtOn ? 1.0 : 0.0);
	return o;
}



)HLSL"
R"HLSL(
float4 psl(VO i) : SV_Target
{
	float3 n = normalize(i.n);
	float3 toCam = camPos.xyz - i.wp;
	float camDist = length(toCam);
	float3 V = toCam / max(camDist, 1e-4);
	float nightF = env.x;
	float dayF = 1.0 - nightF;
	float isTrain = gDraw.x;
	bool glass = gDraw.y > 0.5;
	bool interior = (i.e > 1.5);
	bool lampHead = (i.e > 1.1 && i.e < 1.4);
	float3 lq = float3(0.0, 0.0, 0.0);
	float rm = 0.0;
	if (hlO.w > 0.5) { lq = houseLocal(i.wp); rm = inRoom(lq, 0.04); }
	bool water = (dayF > 0.004) && (isTrain < 0.5) && (abs(i.wp.y + 6.0) < 0.03) && (n.y > 0.98) && (i.c.b > i.c.r * 1.8);
	float3 alb = i.c;
	if (rtG.z > 0.5 && isTrain < 0.5 && !water) alb = surfDetail(i.wp, n, n, camDist, i.c, lerp(1.0, 0.3, rm));
	float3 colDay = float3(0.0, 0.0, 0.0);
	if (dayF > 0.004)
	{
		if (water)
		{
			float ndv = saturate(V.y);
			float fr = saturate(0.06 + 0.98 * pow(1.0 - ndv, 5.0));
			float3 R = reflect(-V, float3(0.0, 1.0, 0.0));
			R.y = abs(R.y);
			colDay = lerp(i.c * 0.8, skyCol(R), fr);
		}
		else
		{
			float ndl = saturate(dot(n, sunDir.xyz));
			colDay = alb * (hemiAmb(n) * (1.0 - 0.45 * rm) + sunI() * ndl * sunCol() * (1.0 - rm));
		}
	}
	float3 colNight = float3(0.0, 0.0, 0.0);
	if (nightF > 0.004)
	{
		if (i.e > 0.5) colNight = i.c * (0.80 + 0.20 * saturate(n.y)) * float3(1.0, 1.0, 0.96);
		else
		{
			colNight = alb * (0.05 + 0.04 * saturate(n.y)) * float3(0.55, 0.65, 1.0);
			if (env.y > 0.01)
			{
				float sSum = 0.0;
				[loop] for (int h = 0; h < 4; h++)
				{
					if (spd[h].w < 0.5) continue;
					sSum += spotL(sp0[h].xyz, i.wp, n, spd[h].xyz) + spotL(sp1[h].xyz, i.wp, n, spd[h].xyz);
				}
				colNight += alb * sSum * float3(1.0, 0.95, 0.80) * env.y;
				colNight += alb * flashLight(i.wp, n) * float3(1.0, 0.96, 0.85);
			}
		}
	}
	float3 col = lerp(colNight, colDay, dayF);
	if (interior) col = i.c * (0.80 + 0.20 * saturate(n.y)) * float3(1.0, 1.0, 0.96);
	if (hlO.w > 0.5 && !interior && !lampHead && isTrain < 0.5)
	{
		if (glass) col += houseGlow(lq);
		else { col = houseFill(col, alb, n, lq); col += alb * houseLight(i.wp, lq, n); }
	}
	float lampMul = (i.e > 0.5) ? dayF : 1.0;
	if (!interior && !glass && !water && !lampHead && lampI.x > 0.0) col += alb * lampLight(i.wp, n) * lampMul;
	if (lampHead) col = float3(1.0, 0.93, 0.72) * 2.4;
	if (!interior)
	{
		float3 clearC = lerp(float3(0.52, 0.72, 0.96), float3(0.01, 0.015, 0.045), nightF);
		float fa = (1.0 - exp(-camDist * 0.00028)) * 0.92 * smoothstep(0.0, 0.6, dayF);
		col = lerp(col, clearC, fa);
	}
	return float4(shoulder(col), 1.0);
}
)HLSL"
R"HLSL(float linZ(float d) { return 450.0 / (3000.0 - d * 2999.85); }
float4 dns(float4 pos : SV_Position) : SV_Target
{
	int2 ip = int2(pos.xy);
	float4 c0 = gColor.Load(int3(ip, 0));
	float4 a0 = gAlb.Load(int3(ip, 0));
	int R = (int)rtE.x;
	if (a0.a < 0.5 || R < 1) return c0;
	float dep0 = gDepth.Load(ip, 0);
	if (dep0 >= 0.99999) return c0;
	float zc = linZ(dep0);
	float zx1 = linZ(gDepth.Load(ip + int2(1, 0), 0));
	float zx0 = linZ(gDepth.Load(ip - int2(1, 0), 0));
	float zy1 = linZ(gDepth.Load(ip + int2(0, 1), 0));
	float zy0 = linZ(gDepth.Load(ip - int2(0, 1), 0));
	float gx = (abs(zx1 - zc) < abs(zc - zx0)) ? (zx1 - zc) : (zc - zx0);
	float gy = (abs(zy1 - zc) < abs(zc - zy0)) ? (zy1 - zc) : (zc - zy0);
	float tol = 0.004 * zc + 0.6 * (abs(gx) + abs(gy)) + 0.01;
	float inv2s = 0.5 / (rtE.y * rtE.y);
	float3 ac = max(a0.rgb, 0.05);
	float3 sum = min(c0.rgb / ac, 10.0);
	float wsum = 1.0;
	[loop] for (int dy = -R; dy <= R; dy++)
	{
		[loop] for (int dx = -R; dx <= R; dx++)
		{
			if (dx != 0 || dy != 0)
			{
				int2 q = ip + int2(dx, dy);
				float4 an = gAlb.Load(int3(q, 0));
				if (an.a > 0.5)
				{
					float zn = linZ(gDepth.Load(q, 0));
					float wz = exp(-abs(zn - (zc + gx * dx + gy * dy)) / tol);
					float ws = exp(-(float)(dx * dx + dy * dy) * inv2s);
					float3 da = an.rgb - a0.rgb;
					float wa = exp(-dot(da, da) * 8.0);
					float w = wz * ws * wa;
					float3 cn = gColor.Load(int3(q, 0)).rgb;
					sum += min(cn / max(an.rgb, 0.05), 10.0) * w;
					wsum += w;
				}
			}
		}
	}
	return float4(saturate(ac * (sum / wsum)), c0.a);
}



float ssaoAO(int2 ip, float d0, float2 fp)
{
	float2 sdims = float2(1.0 / rtC.y, 1.0 / rtC.z);
	float z0 = linZ(d0);
	float w0 = 1.0 / z0;
	float rpx = clamp(rtF.z * (sdims.y * 0.9153) / z0, 2.0, 36.0);
	float jit = ign(fp), jit2 = ign(fp + float2(11.0, 29.0));
	int2 lim = int2(sdims) - 1;
	float occ = 0.0;
	[unroll] for (int k = 0; k < 4; k++)
	{
		float a = ((float)k + jit) * 0.7853982;
		float r = rpx * (0.45 + 0.55 * frac(jit2 + (float)k * 0.618));
		int2 o = int2(round(float2(cos(a), sin(a)) * r));
		float z1 = linZ(gDepth.Load(clamp(ip + o, int2(0, 0), lim), 0));
		float z2 = linZ(gDepth.Load(clamp(ip - o, int2(0, 0), lim), 0));
		float diffm = (0.5 * (1.0 / z1 + 1.0 / z2) - w0) * z0 * z0;
		occ += saturate(diffm / (0.6 * rtF.z)) * ((diffm < 2.5 * rtF.z) ? 1.0 : 0.0);
	}
	float fade = 1.0 - smoothstep(25.0, 90.0, z0);
	return 1.0 - rtF.y * occ * 0.25 * fade;
}

// ---- 疑似レイトレーシング（画面内の深度バッファを使った光線追跡）----
float3 pzWorldAt(int2 pix)
{
	float2 ndc = float2((pix.x + 0.5) * rtC.y * 2.0 - 1.0, 1.0 - (pix.y + 0.5) * rtC.z * 2.0);
	float4 w = mul(float4(ndc, gDepth.Load(pix, 0), 1.0), invVp);
	return w.xyz / w.w;
}
bool pzToPix(float3 p, out int2 pix, out float zv)
{
	pix = int2(0, 0); zv = 0.0;
	float4 c = mul(float4(p, 1.0), vp);
	if (c.w <= 0.001) return false;
	float2 uv = float2(c.x / c.w * 0.5 + 0.5, 0.5 - c.y / c.w * 0.5);
	if (uv.x < 0.0 || uv.y < 0.0 || uv.x >= 1.0 || uv.y >= 1.0) return false;
	pix = int2(uv * float2(1.0 / rtC.y, 1.0 / rtC.z));
	zv = linZ(c.z / c.w);
	return true;
}
float3 pzNormal(int2 ip, float3 p0)
{
	int2 lim = int2(1.0 / rtC.y, 1.0 / rtC.z) - 1;
	float3 pr = pzWorldAt(min(ip + int2(1, 0), lim));
	float3 pl = pzWorldAt(max(ip - int2(1, 0), int2(0, 0)));
	float3 pd = pzWorldAt(min(ip + int2(0, 1), lim));
	float3 pu = pzWorldAt(max(ip - int2(0, 1), int2(0, 0)));
	float3 dx = (length(pr - p0) < length(p0 - pl)) ? (pr - p0) : (p0 - pl);
	float3 dy = (length(pd - p0) < length(p0 - pu)) ? (pd - p0) : (p0 - pu);
	float3 n = cross(dx, dy);
	if (dot(n, n) < 1e-12) return float3(0.0, 1.0, 0.0);
	n = normalize(n);
	if (dot(n, camPos.xyz - p0) < 0.0) n = -n;
	return n;
}
float pzShadow(float3 P, float3 L)
{
	[loop] for (int k = 0; k < 20; k++)
	{
		float3 q = P + L * (0.2 + 0.9 * (float)k);
		int2 pix; float zv;
		if (!pzToPix(q, pix, zv)) return 0.0;
		float gap = zv - linZ(gDepth.Load(pix, 0));
		if (gap > 0.1 && gap < 2.5) return 1.0;
	}
	return 0.0;
}
float4 pzReflect(float3 P, float3 R)
{
	float t = 0.3;
	[loop] for (int k = 0; k < 24; k++)
	{
		float3 q = P + R * t;
		int2 pix; float zv;
		if (!pzToPix(q, pix, zv)) break;
		float gap = zv - linZ(gDepth.Load(pix, 0));
		if (gap > 0.0 && gap < 1.5) return float4(gColor.Load(int3(pix, 0)).rgb, 1.0);
		t *= 1.17;
	}
	return float4(0.0, 0.0, 0.0, 0.0);
}

float4 pps(float4 pos : SV_Position) : SV_Target
{
	int2 ip = int2(pos.xy);
	float4 c0 = gColor.Load(int3(ip, 0));
	float3 col = c0.rgb;
	float2 uv = pos.xy * rtC.yz;
	float2 ndc = float2(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);
	float dep0 = gDepth.Load(ip, 0);
	if (rtC.x > 0.5)
	{
		float4 w4 = mul(float4(ndc, dep0, 1.0), invVp);
		float3 wp = w4.xyz / w4.w;
		float4 pc = mul(float4(wp, 1.0), prevVp);
		float2 vel = float2(0, 0);
		if (pc.w > 1e-3)
		{
			float2 uvp = float2(pc.x / pc.w * 0.5 + 0.5, 0.5 - pc.y / pc.w * 0.5);
			vel = (uvp - uv) * rtD.z;
		}
		float len = length(vel);
		if (len > rtD.w) vel *= rtD.w / len;
		vel *= c0.a;
		if (len > 1e-5)
		{
			float3 acc = float3(0, 0, 0);
			float jit = ign(pos.xy) - 0.5;
			[unroll] for (int s = 0; s < 12; s++)
			{
				float t = ((float)s + 0.5 + jit) / 12.0 - 0.5;
				acc += gColor.SampleLevel(gSamp, uv + vel * t, 0).rgb;
			}
			col = acc / 12.0;
		}
	}


	{
		float sf = 0.0;
		[unroll] for (int ks = 0; ks < 4; ks++) sf += (gDepth.Load(ip, ks) >= 0.99999) ? 0.25 : 0.0;
		if (sf > 0.0)
		{
			float4 wf = mul(float4(ndc, 1.0, 1.0), invVp);
			float3 sdir = normalize(wf.xyz / wf.w - camPos.xyz);
			float3 clearC = lerp(float3(0.52, 0.72, 0.96), float3(0.01, 0.015, 0.045), sunDir.w);
			col += sf * (skyFull(sdir) - clearC);
		}
	}
	if (dep0 < 0.99999 && rtF.y > 0.01) col *= ssaoAO(ip, dep0, pos.xy);
	if (rtH.z > 0.5 && dep0 < 0.99999)
	{
		const float3 P = pzWorldAt(ip);
		const float3 N = pzNormal(ip, P);
		const float3 Vd = normalize(P - camPos.xyz);
		const float sunK = saturate(sunDir.y * 6.0) * saturate(1.0 - env.x);
		if (sunK > 0.001)
		{
			const float ndl = saturate(dot(N, sunDir.xyz));
			if (ndl > 0.0) col *= 1.0 - 0.55 * ndl * sunK * pzShadow(P + N * 0.1, sunDir.xyz);
		}
		if (c0.a > 0.5)
		{
			const float4 rf = pzReflect(P + N * 0.1, reflect(Vd, N));
			const float F = 0.04 + 0.96 * pow(1.0 - saturate(dot(N, -Vd)), 5.0);
			col = lerp(col, rf.rgb, rf.a * F * 0.6);
		}
	}


	if (rtF.w > 0.01)
	{
		float rs = 1.0 / (rtC.z * 720.0);
		float jb = ign(pos.xy) * 0.52;
		float3 bl = float3(0, 0, 0);
		[unroll] for (int kb = 0; kb < 12; kb++)
		{
			float ab = (float)kb * 0.5236 + jb;
			float rb = ((kb < 6) ? 7.0 : 20.0) * rs;
			float2 offb = float2(cos(ab) * rtC.y, sin(ab) * rtC.z) * rb;
			bl += max(gColor.SampleLevel(gSamp, uv + offb, 0).rgb - 0.82, 0.0);
		}
		col += bl * (1.0 / 12.0) * rtF.w * 4.0;
	}


	if (rtG.x > 0.5 && sunDir.w < 0.95 && sunDir.y > -0.05)
	{
		float4 scp = mul(float4(camPos.xyz + sunDir.xyz * 1000.0, 1.0), vp);
		if (scp.w > 0.01)
		{
			float2 sn2 = scp.xy / scp.w;
			float2 suv2 = float2(sn2.x * 0.5 + 0.5, 0.5 - sn2.y * 0.5);
			float2 dl = suv2 - uv;
			float dsun = length(dl * float2(rtC.w, 1.0));
			if (dsun < 1.1)
			{
				float2 sdims = float2(1.0 / rtC.y, 1.0 / rtC.z);
				float2 stp = dl / 14.0;
				float2 pp = uv + stp * ign(pos.xy);
				float acc2 = 0.0, wg = 1.0;
				[unroll] for (int kg = 0; kg < 14; kg++)
				{
					float2 pc2 = clamp(pp, 0.002, 0.998);
					acc2 += ((gDepth.Load(int2(pc2 * sdims), 0) >= 0.99999) ? 1.0 : 0.0) * wg;
					wg *= 0.93;
					pp += stp;
				}
				col += float3(1.0, 0.88, 0.65) * (acc2 / 14.0) * saturate(1.0 - dsun * 0.9) * 0.16 * (1.0 - sunDir.w) * saturate(sunDir.y * 8.0 + 0.4);
			}
		}
	}
	if (rtB.w > 0.5 && sunDir.w < 0.95 && sunDir.y > 0.0)
	{
		float4 sc = mul(float4(camPos.xyz + sunDir.xyz * 1000.0, 1.0), vp);
		if (sc.w > 0.01)
		{
			float2 sn = sc.xy / sc.w;
			float2 suv = float2(sn.x * 0.5 + 0.5, 0.5 - sn.y * 0.5);
			float asp = rtC.w;
			float2 q = (uv - 0.5) * float2(asp, 1.0);
			float2 sq = (suv - 0.5) * float2(asp, 1.0);
			float2 dims = float2(1.0 / rtC.y, 1.0 / rtC.z);
			float vis = 0.0;
			[unroll] for (int k = 0; k < 8; k++)
			{
				float a = k * 0.785398;
				float2 tp = clamp(suv + float2(cos(a), sin(a)) * 0.012, 0.002, 0.998);
				vis += (gDepth.Load(int2(tp * dims), 0) >= 0.99999) ? 1.0 : 0.0;
			}
			vis /= 8.0;
			float edge = saturate(1.6 - max(abs(sn.x), abs(sn.y)));
			float d = length(q - sq);
			float3 f = float3(0, 0, 0);
			f += float3(1.0, 0.92, 0.75) * (0.55 * exp(-d * 9.0) + 0.9 * exp(-d * 55.0));
			f += float3(1.0, 0.95, 0.85) * (1.0 - smoothstep(0.0, 0.035, d)) * 2.0;
			f += float3(1.0, 0.85, 0.60) * exp(-abs(q.y - sq.y) * 70.0) * exp(-abs(q.x - sq.x) * 2.5) * 0.35;
			const float gk[5] = { -0.3, -0.7, -1.1, 0.45, -1.7 };
			const float gr[5] = { 0.045, 0.08, 0.03, 0.11, 0.06 };
			const float3 gc[5] = { float3(1.0, 0.6, 0.3), float3(0.4, 0.8, 1.0), float3(0.9, 0.4, 0.9), float3(0.5, 1.0, 0.6), float3(1.0, 0.8, 0.4) };
			[unroll] for (int g = 0; g < 5; g++)
			{
				float dg = length(q - sq * gk[g]);
				float ring = (1.0 - smoothstep(gr[g] * 0.55, gr[g], dg)) * 0.10 + exp(-sqr((dg - gr[g] * 0.8) / (gr[g] * 0.12))) * 0.10;
				f += gc[g] * ring;
			}
			f += float3(0.9, 0.8, 1.0) * exp(-sqr((d - 0.28) / 0.02)) * 0.12;
			col += f * vis * edge * (1.0 - sunDir.w) * saturate(sunDir.y * 10.0);
		}
	}


	{
		col = saturate(col * 1.06);
		float lum = dot(col, float3(0.299, 0.587, 0.114));
		col = lerp(col * float3(0.97, 0.995, 1.04), col * float3(1.03, 1.0, 0.96), smoothstep(0.25, 0.85, lum));
		col = saturate(col);
		col = lerp(col, col * col * (3.0 - 2.0 * col), 0.22);
		float l2 = dot(col, float3(0.299, 0.587, 0.114));
		col = saturate(lerp(float3(l2, l2, l2), col, 1.20));
		float2 vq = uv - 0.5;
		col *= 1.0 - 0.35 * dot(vq, vq);
	}
	return float4(saturate(col), 1.0);
}
)HLSL";


static const char* kHudShader = R"(
cbuffer HB : register(b0) { float4 scr; };
struct HI { float2 p : POSITION; float4 c : COLOR; };
struct HO { float4 p : SV_POSITION; float4 c : COLOR; };
HO hvs(HI i) {
	HO o;
	o.p = float4(i.p.x / scr.x * 2 - 1, 1 - i.p.y / scr.y * 2, 0, 1);
	o.c = i.c;
	return o;
}
float4 hps(HO i) : SV_Target { return i.c; }

Texture2D tex : register(t5);
float4 ovs(uint id : SV_VertexID) : SV_Position {
	float2 uv = float2((id << 1) & 2, id & 2);
	return float4(uv.x * 2 - 1, 1 - uv.y * 2, 0, 1);
}
float4 ops(float4 p : SV_Position) : SV_Target {
	float3 c = tex.Load(int3(p.xy, 0)).rgb;
	float a = max(c.r, max(c.g, c.b));
	return float4(c, a);
}
)";


static const wchar_t* GFX_NAME[7] = { L"オフ", L"最低", L"低", L"中", L"高", L"最高", L"狂気" };
struct GfxParam {
	float shRays, sunR, aoRays, aoDist, giRays, refl, maxDist, flare, blur, cell, lamp, dnRad, dnSig;
	float ssao, bloom, shafts, cloud, detail, water, vol, bounce2;
};
static const GfxParam GFX[7] = {
	{  0, 0.000f, 0, 0.0f, 0, 0,    0, 0, 0, 2, 0,   0, 0.0f,  0.00f, 0.00f, 0, 0, 2, 0, 0, 0 },

	{  1, 0.000f, 0, 0.0f, 0, 0,  350, 1, 1, 2, 0,   0, 0.0f,  0.30f, 0.30f, 1, 1, 2, 1, 0, 0 },
	{  2, 0.045f, 2, 3.5f, 0, 0,  800, 1, 1, 2, 1,   2, 1.6f,  0.35f, 0.35f, 1, 2, 3, 1, 0, 0 },
	{  3, 0.025f, 2, 4.5f, 2, 1, 1400, 1, 1, 1, 1,   2, 1.3f,  0.00f, 0.40f, 1, 2, 3, 2, 0, 0 },
	{  4, 0.005f, 4, 2.5f, 6, 1, 3000, 1, 1, 1, 1,   4, 1.6f,  0.00f, 0.45f, 1, 2, 3, 2, 0, 0 },
	{  5, 0.004f, 5, 3.0f, 8, 1, 3000, 1, 1, 1, 1,   4, 1.5f,  0.00f, 0.50f, 1, 2, 3, 2, 4, 0 },
	{  6, 0.003f, 6, 3.5f, 6, 1, 3000, 1, 1, 1, 1,   4, 1.4f,  0.00f, 0.55f, 1, 2, 3, 2, 6, 1 },
};
// 疑似レイトレーシング: BVHの光線追跡は行わず、画面内の深度と色から影と反射を光線追跡する
static const GfxParam GFX_PSEUDO = { 0, 0.000f, 0, 0.0f, 0, 0, 0, 1, 1, 2, 1, 0, 0.0f, 0.40f, 0.35f, 1, 2, 3, 2, 0, 0 };
static bool g_pseudoRT = false;
static int  g_gfx = 0;
static bool g_hwRT = false;
static bool g_swGpu = false;
static bool g_useDXR = false, g_useDXC = false;
static bool g_heavyFailed = false;   // 「最低」以降用のシェーダー/PSOを作成済みか・作成に失敗したか
static std::wstring g_gpuName, g_rtNote;


typedef std::vector<unsigned char> Bytes;
static const UINT BB_COUNT = 2;
static ID3D12Device* g_dev; static ID3D12Device5* g_dev5; static ID3D12CommandQueue* g_q; static IDXGISwapChain3* g_sc;
static ID3D12CommandAllocator* g_alloc; static ID3D12GraphicsCommandList* g_cl; static ID3D12GraphicsCommandList4* g_cl4;
static ID3D12Fence* g_fence; static UINT64 g_fenceVal = 0, g_frameFence = 0; static HANDLE g_fenceEvt;
static ID3D12DescriptorHeap* g_rtvHeap, * g_dsvHeap, * g_srvHeap; static UINT g_rtvInc, g_srvInc;
static ID3D12Resource* g_bb[BB_COUNT], * g_msColor, * g_msDepth, * g_resolved, * g_htex, * g_hudUp;
static ID3D12Resource* g_msAlb, * g_resAlb, * g_dn;
static UINT8* g_hudUpPtr; static UINT g_hudPitch;
static ID3D12RootSignature* g_rsScene, * g_rsPost;
static ID3D12PipelineState* g_psoSceneL, * g_psoGlassL;
static ID3D12PipelineState* g_psoDepth, * g_psoScene, * g_psoGlass, * g_psoPost, * g_psoHud, * g_psoOv, * g_psoDn;
static ID3D12Resource* g_cbUp; static UINT8* g_cbPtr;
static HWND g_hwnd;
static int g_w = 1280, g_h = 720;
static std::vector<ID3D12Resource*> g_pending;
static HDC g_hdc; static HBITMAP g_dib; static unsigned char* g_bits;
static HFONT g_font[4]; static int g_fpx[4];
static float S = 1.0f;
static std::string g_shaderErr;
static std::mutex g_errMtx;
static void addErr(const std::string& s) { std::lock_guard<std::mutex> lk(g_errMtx); g_shaderErr += s; }
static const float SKY_DAY[4] = { 0.52f, 0.72f, 0.96f, 1 };

struct Mesh { ID3D12Resource* res = nullptr; D3D12_VERTEX_BUFFER_VIEW vbv{}; UINT start = 0, n = 0; };

// 前照灯を点灯する編成数 (視点に近い順)
static const int HL_TRAIN_MAX = 4;
// 光源数の上限 (端末のレイトレ方式で切り替え)
static const int LAMP_SW_MAX = 8;    // ソフトウェアRT (Intel Iris Xe など) は 8 個
static const int LAMP_HW_MAX = 16;   // ハードウェアRT (DXR 1.1 対応GPU) は 16 個 (シェーダー側 lampP[16] と一致させる)

struct FrameCB {
	M4 vp, invVp, prevVp;
	float camPos[4], sunDir[4], env[4], sp0[HL_TRAIN_MAX][4], sp1[HL_TRAIN_MAX][4], spd[HL_TRAIN_MAX][4], spp[4];
	float rtA[4], rtB[4], rtC[4], rtD[4], rtE[4], rtF[4], rtG[4], rtH[4];
	uint32_t roots[4], triBase[4], carType[4];
	M4 carW[4], carInv[4];
	float lampP[LAMP_HW_MAX][4];
	float lampI[4];
	float flA[4], flB[4], flC[4];
	float hlO[4], hlRot[4];
	float hlP[4][4], hlC[4][4], hlMn[4][4], hlMx[4][4];
};
static_assert(sizeof(FrameCB) % 16 == 0, "FrameCB size");


static D3D12_HEAP_PROPERTIES heapP(D3D12_HEAP_TYPE t) { D3D12_HEAP_PROPERTIES h{}; h.Type = t; h.CreationNodeMask = 1; h.VisibleNodeMask = 1; return h; }
static D3D12_RESOURCE_DESC bufDesc(UINT64 size, D3D12_RESOURCE_FLAGS f = D3D12_RESOURCE_FLAG_NONE) {
	D3D12_RESOURCE_DESC d{}; d.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER; d.Width = size; d.Height = 1; d.DepthOrArraySize = 1; d.MipLevels = 1;
	d.Format = DXGI_FORMAT_UNKNOWN; d.SampleDesc.Count = 1; d.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR; d.Flags = f; return d;
}
static D3D12_RESOURCE_DESC texDesc(UINT w, UINT h, DXGI_FORMAT fmt, UINT samples, D3D12_RESOURCE_FLAGS f) {
	D3D12_RESOURCE_DESC d{}; d.Dimension = D3D12_RESOURCE_DIMENSION_TEXTURE2D; d.Width = w; d.Height = h; d.DepthOrArraySize = 1; d.MipLevels = 1;
	d.Format = fmt; d.SampleDesc.Count = samples; d.Layout = D3D12_TEXTURE_LAYOUT_UNKNOWN; d.Flags = f; return d;
}
static ID3D12Resource* createBuf(UINT64 size, D3D12_HEAP_TYPE ht, D3D12_RESOURCE_STATES st, D3D12_RESOURCE_FLAGS f = D3D12_RESOURCE_FLAG_NONE) {
	ID3D12Resource* r = nullptr;
	D3D12_HEAP_PROPERTIES hp = heapP(ht); D3D12_RESOURCE_DESC d = bufDesc(size, f);
	if (FAILED(g_dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, st, nullptr, IID_PPV_ARGS(&r)))) return nullptr;
	return r;
}
static D3D12_RESOURCE_BARRIER trans(ID3D12Resource* r, D3D12_RESOURCE_STATES a, D3D12_RESOURCE_STATES b) {
	D3D12_RESOURCE_BARRIER x{}; x.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION; x.Transition.pResource = r;
	x.Transition.Subresource = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES; x.Transition.StateBefore = a; x.Transition.StateAfter = b; return x;
}
static D3D12_RESOURCE_BARRIER uavB(ID3D12Resource* r) { D3D12_RESOURCE_BARRIER x{}; x.Type = D3D12_RESOURCE_BARRIER_TYPE_UAV; x.UAV.pResource = r; return x; }
static D3D12_CPU_DESCRIPTOR_HANDLE rtvAt(UINT i) { auto h = g_rtvHeap->GetCPUDescriptorHandleForHeapStart(); h.ptr += (SIZE_T)i * g_rtvInc; return h; }
static D3D12_CPU_DESCRIPTOR_HANDLE srvCpu(UINT i) { auto h = g_srvHeap->GetCPUDescriptorHandleForHeapStart(); h.ptr += (SIZE_T)i * g_srvInc; return h; }
static D3D12_GPU_DESCRIPTOR_HANDLE srvGpu(UINT i) { auto h = g_srvHeap->GetGPUDescriptorHandleForHeapStart(); h.ptr += (UINT64)i * g_srvInc; return h; }

static void waitFence(UINT64 v) {
	if (g_fence->GetCompletedValue() < v) { g_fence->SetEventOnCompletion(v, g_fenceEvt); WaitForSingleObject(g_fenceEvt, INFINITE); }
}
static void gpuIdle() { g_fenceVal++; g_q->Signal(g_fence, g_fenceVal); waitFence(g_fenceVal); g_frameFence = g_fenceVal; }


static void flushInit() {
	g_cl->Close();
	ID3D12CommandList* l[] = { g_cl };
	g_q->ExecuteCommandLists(1, l);
	gpuIdle();
	for (auto* r : g_pending) r->Release();
	g_pending.clear();
	g_alloc->Reset();
}


static ID3D12Resource* makeStaticBuffer(const void* data, size_t bytes) {
	if (bytes == 0) bytes = 16;
	ID3D12Resource* def = createBuf(bytes, D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_COMMON);
	ID3D12Resource* up = createBuf(bytes, D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ);
	if (!def || !up) return nullptr;
	void* p = nullptr; up->Map(0, nullptr, &p);
	if (data) memcpy(p, data, bytes); else memset(p, 0, bytes);
	up->Unmap(0, nullptr);
	D3D12_RESOURCE_BARRIER b0 = trans(def, D3D12_RESOURCE_STATE_COMMON, D3D12_RESOURCE_STATE_COPY_DEST);
	g_cl->ResourceBarrier(1, &b0);
	g_cl->CopyBufferRegion(def, 0, up, 0, bytes);
	D3D12_RESOURCE_BARRIER b1 = trans(def, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_GENERIC_READ);
	g_cl->ResourceBarrier(1, &b1);
	g_pending.push_back(up);
	return def;
}
static Mesh makeMesh(const Verts& v) {
	Mesh m; m.n = (UINT)v.size();
	size_t bytes = std::max<size_t>(v.size() * sizeof(Vtx), 16);
	m.res = makeStaticBuffer(v.data(), v.size() * sizeof(Vtx));
	m.vbv.BufferLocation = m.res->GetGPUVirtualAddress(); m.vbv.SizeInBytes = (UINT)bytes; m.vbv.StrideInBytes = sizeof(Vtx);
	return m;
}
static Mesh subMesh(const Mesh& m, UINT start, UINT n) { Mesh r = m; r.start = start; r.n = n; return r; }



struct BNode { float mn[3]; uint32_t a; float mx[3]; uint32_t cnt; };
struct BTri { V3 mn, mx, c; };
static float bAxis(V3 v, int ax) { return ax == 0 ? v.x : (ax == 1 ? v.y : v.z); }
static float bArea(V3 mn, V3 mx) {
	float dx = mx.x - mn.x, dy = mx.y - mn.y, dz = mx.z - mn.z;
	if (dx < 0 || dy < 0 || dz < 0) return 0.0f;
	return 2.0f * (dx * dy + dy * dz + dz * dx);
}
static V3 bMin(V3 a, V3 b) { return { std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z) }; }
static V3 bMax(V3 a, V3 b) { return { std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z) }; }

static uint32_t bvhRec(std::vector<BNode>& nodes, std::vector<uint32_t>& idx, const std::vector<BTri>& bt, uint32_t lo, uint32_t hi, int depth) {
	const uint32_t ni = (uint32_t)nodes.size();
	nodes.push_back(BNode{});
	const float BIG = 1e30f;
	V3 mn = { BIG, BIG, BIG }, mx = { -BIG, -BIG, -BIG }, cmn = mn, cmx = mx;
	for (uint32_t i = lo; i < hi; i++) {
		const BTri& t = bt[idx[i]];
		mn = bMin(mn, t.mn); mx = bMax(mx, t.mx); cmn = bMin(cmn, t.c); cmx = bMax(cmx, t.c);
	}
	const uint32_t cnt = hi - lo;
	BNode nd{};
	nd.mn[0] = mn.x; nd.mn[1] = mn.y; nd.mn[2] = mn.z; nd.mx[0] = mx.x; nd.mx[1] = mx.y; nd.mx[2] = mx.z;
	V3 ext = cmx - cmn;
	int ax = (ext.x >= ext.y && ext.x >= ext.z) ? 0 : (ext.y >= ext.z ? 1 : 2);
	float e = bAxis(ext, ax);
	if (cnt <= 4 || depth >= 48 || (e < 1e-6f && cnt <= 16)) {
		nd.a = lo; nd.cnt = cnt; nodes[ni] = nd; return ni;
	}
	uint32_t mid = (lo + hi) / 2;
	bool done = false;
	if (e >= 1e-6f) {
		const int NB = 12;
		struct Bin { V3 mn, mx; uint32_t n; } bins[NB];
		for (int b = 0; b < NB; b++) { bins[b].mn = { BIG, BIG, BIG }; bins[b].mx = { -BIG, -BIG, -BIG }; bins[b].n = 0; }
		const float cmin = bAxis(cmn, ax);
		auto binOf = [&](const BTri& t) { int b = (int)((bAxis(t.c, ax) - cmin) / e * NB); return std::min(NB - 1, std::max(0, b)); };
		for (uint32_t i = lo; i < hi; i++) {
			const BTri& t = bt[idx[i]]; Bin& b = bins[binOf(t)];
			b.mn = bMin(b.mn, t.mn); b.mx = bMax(b.mx, t.mx); b.n++;
		}
		float rArea[NB]; uint32_t rN[NB];
		V3 amn = { BIG, BIG, BIG }, amx = { -BIG, -BIG, -BIG }; uint32_t an = 0;
		for (int b = NB - 1; b >= 1; b--) { amn = bMin(amn, bins[b].mn); amx = bMax(amx, bins[b].mx); an += bins[b].n; rArea[b] = bArea(amn, amx); rN[b] = an; }
		amn = { BIG, BIG, BIG }; amx = { -BIG, -BIG, -BIG }; an = 0;
		float best = BIG; int bs = -1;
		for (int b = 0; b < NB - 1; b++) {
			amn = bMin(amn, bins[b].mn); amx = bMax(amx, bins[b].mx); an += bins[b].n;
			if (an == 0 || rN[b + 1] == 0) continue;
			float c = an * bArea(amn, amx) + rN[b + 1] * rArea[b + 1];
			if (c < best) { best = c; bs = b + 1; }
		}
		if (bs >= 0) {
			uint32_t* base = idx.data();
			uint32_t* m = std::partition(base + lo, base + hi, [&](uint32_t k) { return binOf(bt[k]) < bs; });
			mid = (uint32_t)(m - base);
			done = (mid > lo && mid < hi);
		}
	}
	if (!done) {
		mid = (lo + hi) / 2;
		std::nth_element(idx.begin() + lo, idx.begin() + mid, idx.begin() + hi,
			[&](uint32_t a, uint32_t b) { return bAxis(bt[a].c, ax) < bAxis(bt[b].c, ax); });
	}
	if (depth < 3 && cnt >= 30000) {

		std::vector<BNode> L, Rv;
		auto fut = std::async(std::launch::async, [&]() { bvhRec(L, idx, bt, lo, mid, depth + 1); });
		bvhRec(Rv, idx, bt, mid, hi, depth + 1);
		fut.get();
		const uint32_t lbase = (uint32_t)nodes.size();
		for (BNode n : L) { if (n.cnt == 0) n.a += lbase; nodes.push_back(n); }
		const uint32_t rbase = (uint32_t)nodes.size();
		for (BNode n : Rv) { if (n.cnt == 0) n.a += rbase; nodes.push_back(n); }
		nd.a = rbase; nd.cnt = 0; nodes[ni] = nd;
		return ni;
	}
	bvhRec(nodes, idx, bt, lo, mid, depth + 1);
	uint32_t r = bvhRec(nodes, idx, bt, mid, hi, depth + 1);
	nd.a = r; nd.cnt = 0; nodes[ni] = nd;
	return ni;
}


static void packTri(std::vector<float>& out, const Verts& v, size_t t) {
	const Vtx& a = v[t * 3]; const Vtx& b = v[t * 3 + 1]; const Vtx& c = v[t * 3 + 2];
	V3 col = (a.c + b.c + c.c) * (1.0f / 3.0f);
	auto q = [](float x) { return (uint32_t)std::min(255.0f, std::max(0.0f, x * 255.0f + 0.5f)); };
	uint32_t pk = q(col.x) | (q(col.y) << 8) | (q(col.z) << 16) | (a.e > 0.5f ? 0xFF000000u : 0u);
	float pkf; memcpy(&pkf, &pk, 4);
	const float d[12] = { a.p.x, a.p.y, a.p.z, pkf, b.p.x, b.p.y, b.p.z, 0.0f, c.p.x, c.p.y, c.p.z, 0.0f };
	out.insert(out.end(), d, d + 12);
}

static void buildBVHLocal(const Verts& v, std::vector<BNode>& nodes, std::vector<float>& tris) {
	const size_t nt = v.size() / 3;
	std::vector<BTri> bt(nt); std::vector<uint32_t> idx(nt);
	for (size_t t = 0; t < nt; t++) {
		const V3& a = v[t * 3].p; const V3& b = v[t * 3 + 1].p; const V3& c = v[t * 3 + 2].p;
		bt[t].mn = bMin(a, bMin(b, c)); bt[t].mx = bMax(a, bMax(b, c));
		bt[t].c = (bt[t].mn + bt[t].mx) * 0.5f; idx[t] = (uint32_t)t;
	}
	nodes.clear(); nodes.reserve(nt / 2 + 4);
	bvhRec(nodes, idx, bt, 0, (uint32_t)nt, 0);
	tris.clear(); tris.reserve(nt * 12);
	for (size_t k = 0; k < nt; k++) packTri(tris, v, idx[k]);
}

static uint32_t mergeBVH(std::vector<BNode>& nodes, std::vector<float>& tris, const std::vector<BNode>& ln, const std::vector<float>& lt) {
	const uint32_t nodeBase = (uint32_t)nodes.size(), triBase = (uint32_t)(tris.size() / 12);
	for (BNode n : ln) { if (n.cnt) n.a += triBase; else n.a += nodeBase; nodes.push_back(n); }
	tris.insert(tris.end(), lt.begin(), lt.end());
	return nodeBase;
}

static void appendTrisOnly(std::vector<float>& tris, const Verts& v) {
	const size_t nt = v.size() / 3;
	tris.reserve(tris.size() + nt * 12);
	for (size_t t = 0; t < nt; t++) packTri(tris, v, t);
}

static M4 inverse4(const M4& a) {
	const float* m = &a.m[0][0]; float inv[16];
	inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
	inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
	inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
	inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
	inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
	inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
	inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
	inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
	inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
	inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
	inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
	inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
	inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
	inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
	inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
	inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];
	float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
	M4 r = ident();
	if (fabsf(det) < 1e-20f) return r;
	det = 1.0f / det;
	for (int i = 0; i < 16; i++) (&r.m[0][0])[i] = inv[i] * det;
	return r;
}



#ifdef HAVE_DXC
static HMODULE g_dxcMod = nullptr; static DxcCreateInstanceProc g_dxcCreate = nullptr;
static const GUID kClsidDxcCompiler = { 0x73e22d93, 0xe6ce, 0x47f3, { 0xb5, 0xbf, 0xf0, 0x66, 0x4f, 0x39, 0xc1, 0xb0 } };
static const GUID kClsidDxcUtils = { 0x6245d6af, 0x66e0, 0x48fd, { 0x80, 0xb4, 0x4d, 0x27, 0x17, 0x96, 0x74, 0x8c } };
static bool tryDxcDir(const std::wstring& d) {
	if (GetFileAttributesW((d + L"dxcompiler.dll").c_str()) == INVALID_FILE_ATTRIBUTES) return false;
	LoadLibraryW((d + L"dxil.dll").c_str());
	HMODULE m = LoadLibraryW((d + L"dxcompiler.dll").c_str());
	if (!m) return false;
	auto f = (DxcCreateInstanceProc)GetProcAddress(m, "DxcCreateInstance");
	if (!f) return false;
	g_dxcMod = m; g_dxcCreate = f; return true;
}

static bool loadDXC() {
	wchar_t path[MAX_PATH]; GetModuleFileNameW(nullptr, path, MAX_PATH);
	std::wstring dir = path; dir = dir.substr(0, dir.find_last_of(L"\\/") + 1);
	if (tryDxcDir(dir)) return true;
	wchar_t pf[MAX_PATH]; DWORD n = GetEnvironmentVariableW(L"ProgramFiles(x86)", pf, MAX_PATH);
	if (n > 0 && n < MAX_PATH) {
		std::wstring base = std::wstring(pf) + L"\\Windows Kits\\10\\bin\\";
		WIN32_FIND_DATAW fd; HANDLE h = FindFirstFileW((base + L"*").c_str(), &fd);
		std::vector<std::wstring> vers;
		if (h != INVALID_HANDLE_VALUE) {
			do { if ((fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) && fd.cFileName[0] == L'1') vers.push_back(fd.cFileName); } while (FindNextFileW(h, &fd));
			FindClose(h);
		}
		std::sort(vers.rbegin(), vers.rend());
		for (auto& v : vers) if (tryDxcDir(base + v + L"\\x64\\")) return true;
	}
	HMODULE m = LoadLibraryW(L"dxcompiler.dll");
	if (m) { auto f = (DxcCreateInstanceProc)GetProcAddress(m, "DxcCreateInstance"); if (f) { g_dxcMod = m; g_dxcCreate = f; return true; } }
	return false;
}
#else
static bool loadDXC() { return false; }
#endif

static bool compileShader(const char* src, const char* entry, const char* kind, Bytes& out, bool dxr) {
	std::string err;
#ifdef HAVE_DXC
	if (dxr) {
		IDxcUtils* utils = nullptr; IDxcCompiler3* comp = nullptr;
		if (FAILED(g_dxcCreate(kClsidDxcUtils, IID_PPV_ARGS(&utils))) || FAILED(g_dxcCreate(kClsidDxcCompiler, IID_PPV_ARGS(&comp)))) { addErr("DXC初期化失敗\n"); return false; }
		std::wstring e(entry, entry + strlen(entry)), prof = std::wstring(kind, kind + 2) + L"_6_5";
		std::vector<LPCWSTR> args = { L"-E", e.c_str(), L"-T", prof.c_str(), L"-O3", L"-D", dxr ? L"USE_DXR=1" : L"USE_DXR=0" };
		DxcBuffer buf{ src, strlen(src), DXC_CP_UTF8 };
		IDxcResult* res = nullptr;
		HRESULT hr = comp->Compile(&buf, args.data(), (UINT32)args.size(), nullptr, IID_PPV_ARGS(&res));
		bool ok = false;
		if (SUCCEEDED(hr) && res) {
			HRESULT st = E_FAIL; res->GetStatus(&st);
			IDxcBlobUtf8* errs = nullptr; res->GetOutput(DXC_OUT_ERRORS, IID_PPV_ARGS(&errs), nullptr);
			if (errs && errs->GetStringLength() > 0) addErr(std::string("[DXC ") + entry + "] " + errs->GetStringPointer() + "\n");
			if (SUCCEEDED(st)) {
				IDxcBlob* obj = nullptr; res->GetOutput(DXC_OUT_OBJECT, IID_PPV_ARGS(&obj), nullptr);
				if (obj && obj->GetBufferSize() > 0) { out.assign((unsigned char*)obj->GetBufferPointer(), (unsigned char*)obj->GetBufferPointer() + obj->GetBufferSize()); ok = true; }
				if (obj) obj->Release();
			}
			if (errs) errs->Release();
			res->Release();
		}
		comp->Release(); utils->Release();
		return ok;
	}
#endif
	D3D_SHADER_MACRO mac[] = { { "USE_DXR", "0" }, { nullptr, nullptr } };
	std::string prof = std::string(kind) + "_5_1";
	ID3DBlob* b = nullptr, * e = nullptr;
	HRESULT hr = D3DCompile(src, strlen(src), nullptr, mac, nullptr, entry, prof.c_str(), D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &b, &e);
	if (e) { addErr(std::string("[FXC ") + entry + "] " + (const char*)e->GetBufferPointer() + "\n"); e->Release(); }
	if (FAILED(hr) || !b) return false;
	out.assign((unsigned char*)b->GetBufferPointer(), (unsigned char*)b->GetBufferPointer() + b->GetBufferSize());
	b->Release();
	return true;
}
static Bytes g_bPSL;
static Bytes g_bVS, g_bPS, g_bPP, g_bDN, g_bHVS, g_bHPS, g_bOVS, g_bOPS;

struct ShJob { const char* src; const char* entry; const char* kind; Bytes* out; };
static bool compileJobs(const ShJob* jobs, size_t n, bool dxr, bool lowPrio = false) {
	std::vector<std::future<bool>> fs;
	for (size_t i = 0; i < n; i++) {
		ShJob j = jobs[i];
		fs.push_back(std::async(std::launch::async, [j, dxr, lowPrio]() {
			if (lowPrio) SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);   // ゲーム本体の描画を邪魔しない
			return compileShader(j.src, j.entry, j.kind, *j.out, dxr);
			}));
	}
	bool ok = true;
	for (auto& f : fs) ok = f.get() && ok;
	return ok;
}
// 起動時に必要な分だけ（グラフィック「オフ」の描画用 vs/psl と、HUD・メニュー用）
static bool compileLite() {
	const ShJob jobs[6] = {
		{ kShader, "vs", "vs", &g_bVS }, { kShader, "psl", "ps", &g_bPSL },
		{ kHudShader, "hvs", "vs", &g_bHVS }, { kHudShader, "hps", "ps", &g_bHPS }, { kHudShader, "ovs", "vs", &g_bOVS }, { kHudShader, "ops", "ps", &g_bOPS } };
	return compileJobs(jobs, 6, g_useDXR);
}
static ID3D12RootSignature* makeRS(bool scene) {
	D3D12_ROOT_PARAMETER p[3]{}; D3D12_DESCRIPTOR_RANGE rg{};
	p[0].ParameterType = D3D12_ROOT_PARAMETER_TYPE_32BIT_CONSTANTS; p[0].Constants.ShaderRegister = 0; p[0].Constants.Num32BitValues = scene ? 20 : 4;
	p[0].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
	p[1].ParameterType = D3D12_ROOT_PARAMETER_TYPE_CBV; p[1].Descriptor.ShaderRegister = 1; p[1].ShaderVisibility = D3D12_SHADER_VISIBILITY_ALL;
	rg.RangeType = D3D12_DESCRIPTOR_RANGE_TYPE_SRV; rg.NumDescriptors = scene ? 3 : 4; rg.BaseShaderRegister = scene ? 0 : 3; rg.OffsetInDescriptorsFromTableStart = D3D12_DESCRIPTOR_RANGE_OFFSET_APPEND;
	p[2].ParameterType = D3D12_ROOT_PARAMETER_TYPE_DESCRIPTOR_TABLE; p[2].DescriptorTable.NumDescriptorRanges = 1; p[2].DescriptorTable.pDescriptorRanges = &rg;
	p[2].ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
	D3D12_STATIC_SAMPLER_DESC ss{}; ss.Filter = D3D12_FILTER_MIN_MAG_MIP_LINEAR;
	ss.AddressU = ss.AddressV = ss.AddressW = D3D12_TEXTURE_ADDRESS_MODE_CLAMP; ss.ComparisonFunc = D3D12_COMPARISON_FUNC_ALWAYS;
	ss.MaxLOD = D3D12_FLOAT32_MAX; ss.ShaderRegister = 0; ss.ShaderVisibility = D3D12_SHADER_VISIBILITY_PIXEL;
	D3D12_ROOT_SIGNATURE_DESC d{}; d.NumParameters = 3; d.pParameters = p; d.NumStaticSamplers = 1; d.pStaticSamplers = &ss;
	d.Flags = D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT;
	ID3DBlob* blob = nullptr, * err = nullptr;
	if (FAILED(D3D12SerializeRootSignature(&d, D3D_ROOT_SIGNATURE_VERSION_1, &blob, &err))) { if (err) err->Release(); return nullptr; }
	ID3D12RootSignature* rs = nullptr;
	g_dev->CreateRootSignature(0, blob->GetBufferPointer(), blob->GetBufferSize(), IID_PPV_ARGS(&rs));
	blob->Release();
	return rs;
}
enum { BL_NONE, BL_ALPHA, BL_PRE, BL_GLASS, BL_MASK0 };
enum { DP_NONE, DP_LE_WRITE, DP_LT_NOWRITE, DP_LT_WRITE };
static ID3D12PipelineState* makePSO(ID3D12RootSignature* rs, const Bytes& vs, const Bytes* ps, const D3D12_INPUT_ELEMENT_DESC* el, UINT nel,
	int blend, int depth, UINT samples, int numRT) {
	D3D12_GRAPHICS_PIPELINE_STATE_DESC d{};
	d.pRootSignature = rs; d.VS = { vs.data(), vs.size() };
	if (ps) d.PS = { ps->data(), ps->size() };
	auto& rt = d.BlendState.RenderTarget[0];
	rt.SrcBlend = D3D12_BLEND_ONE; rt.DestBlend = D3D12_BLEND_ZERO; rt.BlendOp = D3D12_BLEND_OP_ADD;
	rt.SrcBlendAlpha = D3D12_BLEND_ONE; rt.DestBlendAlpha = D3D12_BLEND_ZERO; rt.BlendOpAlpha = D3D12_BLEND_OP_ADD;
	rt.LogicOp = D3D12_LOGIC_OP_NOOP; rt.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_ALL;
	if (blend == BL_ALPHA) { rt.BlendEnable = TRUE; rt.SrcBlend = D3D12_BLEND_SRC_ALPHA; rt.DestBlend = D3D12_BLEND_INV_SRC_ALPHA; }
	else if (blend == BL_PRE) { rt.BlendEnable = TRUE; rt.SrcBlend = D3D12_BLEND_ONE; rt.DestBlend = D3D12_BLEND_INV_SRC_ALPHA; }
	else if (blend == BL_MASK0) { rt.RenderTargetWriteMask = 0; }
	else if (blend == BL_GLASS) {
		rt.BlendEnable = TRUE; rt.SrcBlend = D3D12_BLEND_BLEND_FACTOR; rt.DestBlend = D3D12_BLEND_INV_BLEND_FACTOR;
		rt.RenderTargetWriteMask = D3D12_COLOR_WRITE_ENABLE_RED | D3D12_COLOR_WRITE_ENABLE_GREEN | D3D12_COLOR_WRITE_ENABLE_BLUE;
	}
	if (numRT >= 2) {
		d.BlendState.IndependentBlendEnable = TRUE;
		auto& r1 = d.BlendState.RenderTarget[1];
		r1 = rt; r1.BlendEnable = FALSE; r1.SrcBlend = D3D12_BLEND_ONE; r1.DestBlend = D3D12_BLEND_ZERO;
		r1.RenderTargetWriteMask = (blend == BL_GLASS || blend == BL_MASK0) ? 0 : D3D12_COLOR_WRITE_ENABLE_ALL;
	}
	d.SampleMask = UINT_MAX;
	d.RasterizerState.FillMode = D3D12_FILL_MODE_SOLID; d.RasterizerState.CullMode = D3D12_CULL_MODE_NONE; d.RasterizerState.DepthClipEnable = TRUE;
	auto& ds = d.DepthStencilState;
	ds.DepthEnable = depth != DP_NONE; ds.DepthWriteMask = (depth == DP_LE_WRITE || depth == DP_LT_WRITE) ? D3D12_DEPTH_WRITE_MASK_ALL : D3D12_DEPTH_WRITE_MASK_ZERO;
	ds.DepthFunc = (depth == DP_LE_WRITE) ? D3D12_COMPARISON_FUNC_LESS_EQUAL : D3D12_COMPARISON_FUNC_LESS;
	ds.StencilReadMask = ds.StencilWriteMask = 0xFF;
	ds.FrontFace = { D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_STENCIL_OP_KEEP, D3D12_COMPARISON_FUNC_ALWAYS }; ds.BackFace = ds.FrontFace;
	d.InputLayout = { el, nel };
	d.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
	d.NumRenderTargets = numRT; for (int k = 0; k < numRT; k++) d.RTVFormats[k] = DXGI_FORMAT_R8G8B8A8_UNORM;
	d.DSVFormat = depth != DP_NONE ? DXGI_FORMAT_D32_FLOAT : DXGI_FORMAT_UNKNOWN;
	d.SampleDesc.Count = samples;
	ID3D12PipelineState* pso = nullptr;
	if (FAILED(g_dev->CreateGraphicsPipelineState(&d, IID_PPV_ARGS(&pso)))) return nullptr;
	return pso;
}


static const D3D12_INPUT_ELEMENT_DESC kElScene[4] = {
	{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0,  D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
	{ "NORMAL",   0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
	{ "COLOR",    0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
	{ "TEXCOORD", 0, DXGI_FORMAT_R32_FLOAT,       0, 36, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
};
static const D3D12_INPUT_ELEMENT_DESC kElHud[2] = {
	{ "POSITION", 0, DXGI_FORMAT_R32G32_FLOAT,       0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
	{ "COLOR",    0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 8, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
};
static void relPSO(ID3D12PipelineState*& p) { if (p) { p->Release(); p = nullptr; } }
static std::future<ID3D12PipelineState*> asyncPSO(ID3D12RootSignature* rs, const Bytes& vs, const Bytes* ps, const D3D12_INPUT_ELEMENT_DESC* e, UINT ne, int bl, int dp, UINT sm, int nrt) {
	return std::async(std::launch::async, [=, &vs]() { return makePSO(rs, vs, ps, e, ne, bl, dp, sm, nrt); });
}
// 起動時に作る PSO（「オフ」の描画 + HUD）。作り直す場合は GPU が止まっている時に呼ぶこと
static bool createLitePSOs() {
	auto f4 = asyncPSO(g_rsPost, g_bHVS, &g_bHPS, kElHud, 2, BL_ALPHA, DP_NONE, 1, 1);
	auto f5 = asyncPSO(g_rsPost, g_bOVS, &g_bOPS, nullptr, 0, BL_PRE, DP_NONE, 1, 1);
	auto f7 = asyncPSO(g_rsScene, g_bVS, &g_bPSL, kElScene, 4, BL_NONE, DP_LE_WRITE, 4, 1);
	auto f8 = asyncPSO(g_rsScene, g_bVS, &g_bPSL, kElScene, 4, BL_GLASS, DP_LT_NOWRITE, 4, 1);
	relPSO(g_psoHud); relPSO(g_psoOv); relPSO(g_psoSceneL); relPSO(g_psoGlassL);
	g_psoHud = f4.get(); g_psoOv = f5.get(); g_psoSceneL = f7.get(); g_psoGlassL = f8.get();
	return g_psoHud && g_psoOv && g_psoSceneL && g_psoGlassL;
}
// ============================================================================
// 「最低」以降のグラフィックレベル用シェーダーを、起動後にバックグラウンドで読み込む（コンパイル + PSO作成）。
//  ・ワーカースレッドは「ステージング領域 g_bg」にしか書き込まない。ゲームが使っている g_pso* や g_useDXR 等には一切触れない
//  ・段階1（疑似RTに必要な ps / pps と PSO）が終わった時点で疑似RTを使える。段階2（dn パス）は後から差し替える
//  ・段階ごとに完成した PSO の組（g_setSW / g_setHW）を、フレーム毎に選んで使うだけなので、切り替えは一瞬
// ============================================================================
enum { BG_IDLE, BG_RUNNING, BG_DONE_OK, BG_DONE_FAIL };
static std::atomic<int>  g_bgState{ BG_IDLE };
static std::atomic<bool> g_bgCancel{ false };
static std::thread g_bgThread;
struct BgShaders {
	Bytes vs, ovs, ps, pp, dn;        // 段階1・2: DXBC（SW / 疑似RT 用）
	Bytes hps, hpp, hdn;              // 段階3: DXIL（HW RT 用）
};
static BgShaders g_bg;

// 描画に使う PSO の組。SW組（疑似RT・SW-RT のレベル、DXBC）と HW組（HW-RT のレベル、DXIL）を別々に持つ
struct PsoSet { ID3D12PipelineState* depth = nullptr, * scene = nullptr, * glass = nullptr, * post = nullptr, * dn = nullptr; };
static PsoSet g_setSW, g_setHW;   // ワーカーが書き込んでから、対応するフラグを立てる（メインスレッドは、フラグを見てから読む）

// 読み込み中（ワーカー実行中、または完了したがまだメインスレッドで回収していない）か
static bool shaderLoading() { return g_bgThread.joinable(); }

static std::atomic<bool> g_bgPseudoReady{ false };   // 段階1 完了: 疑似RTが使える
static std::atomic<bool> g_bgSwDone{ false };        // 段階2 完了: SW-RT のレベルが使える（dn が作れなくても完了扱い）
static std::atomic<bool> g_bgHwDone{ false };        // 段階3 完了: HW-RT のレベルが使える
static std::atomic<bool> g_bgHwFailed{ false };      // 段階3 失敗: HW-RT を諦めた（SW-RT で続ける）
static bool g_bgHwFailApplied = false;               // 段階3の失敗をメインスレッドで反映済みか

static void relSet(PsoSet& s) { relPSO(s.depth); relPSO(s.scene); relPSO(s.glass); relPSO(s.post); relPSO(s.dn); }

// 段階1: 疑似RT（最優先）。DXBC のみ（DXC が無くても動く）
// 段階2: SW-RT のぼかし・グロー（dns, DXBC）
// 段階3: HW-RT（DXIL）。HW-RT の GPU でだけ行い、失敗したら SW-RT で続ける
static void bgLoadWorker() {
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);   // ゲーム本体の描画を邪魔しない
	BgShaders& b = g_bg;
	const bool hwWanted = g_useDXR;   // 開始時点の値（メインスレッドは、完了を回収するまで変えない）

	auto mk = [](ID3D12RootSignature* rs, const Bytes& vs, const Bytes* ps, const D3D12_INPUT_ELEMENT_DESC* e, UINT ne, int bl, int dp, UINT sm, int nrt) {
		return std::async(std::launch::async, [=, &vs]() {
			SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
			return makePSO(rs, vs, ps, e, ne, bl, dp, sm, nrt);
			});
		};

	// ---- 段階1（疑似RT）----
	const ShJob sw1[4] = { { kShader, "vs", "vs", &b.vs }, { kHudShader, "ovs", "vs", &b.ovs },
		{ kShader, "ps", "ps", &b.ps }, { kShader, "pps", "ps", &b.pp } };
	if (!compileJobs(sw1, 4, false, true) || g_bgCancel.load()) { g_bgState.store(BG_DONE_FAIL); return; }
	{
		auto f0 = mk(g_rsScene, b.vs, nullptr, kElScene, 4, BL_MASK0, DP_LT_WRITE, 4, 2);
		auto f1 = mk(g_rsScene, b.vs, &b.ps, kElScene, 4, BL_NONE, DP_LE_WRITE, 4, 2);
		auto f2 = mk(g_rsScene, b.vs, &b.ps, kElScene, 4, BL_GLASS, DP_LT_NOWRITE, 4, 2);
		auto f3 = mk(g_rsPost, b.ovs, &b.pp, nullptr, 0, BL_NONE, DP_NONE, 1, 1);
		g_setSW.depth = f0.get(); g_setSW.scene = f1.get(); g_setSW.glass = f2.get(); g_setSW.post = f3.get();
	}
	if (!(g_setSW.depth && g_setSW.scene && g_setSW.glass && g_setSW.post)) { g_bgState.store(BG_DONE_FAIL); return; }
	g_bgPseudoReady.store(true);

	// ---- 段階2（SW-RT のぼかし・グロー）----
	const ShJob dn1[1] = { { kShader, "dns", "ps", &b.dn } };
	if (!g_bgCancel.load() && compileJobs(dn1, 1, false, true) && !g_bgCancel.load()) {
		g_setSW.dn = mk(g_rsPost, b.ovs, &b.dn, nullptr, 0, BL_NONE, DP_NONE, 1, 1).get();
	}
	g_bgSwDone.store(true);
	if (g_bgCancel.load() || !hwWanted) { g_bgState.store(BG_DONE_OK); return; }

	// ---- 段階3（HW-RT、DXIL）----
	const ShJob hw3[3] = { { kShader, "ps", "ps", &b.hps }, { kShader, "pps", "ps", &b.hpp }, { kShader, "dns", "ps", &b.hdn } };
	bool ok = compileJobs(hw3, 3, true, true) && !g_bgCancel.load();
	if (ok) {
		auto h0 = mk(g_rsScene, g_bVS, nullptr, kElScene, 4, BL_MASK0, DP_LT_WRITE, 4, 2);
		auto h1 = mk(g_rsScene, g_bVS, &b.hps, kElScene, 4, BL_NONE, DP_LE_WRITE, 4, 2);
		auto h2 = mk(g_rsScene, g_bVS, &b.hps, kElScene, 4, BL_GLASS, DP_LT_NOWRITE, 4, 2);
		auto h3 = mk(g_rsPost, g_bOVS, &b.hpp, nullptr, 0, BL_NONE, DP_NONE, 1, 1);
		auto h4 = mk(g_rsPost, g_bOVS, &b.hdn, nullptr, 0, BL_NONE, DP_NONE, 1, 1);
		g_setHW.depth = h0.get(); g_setHW.scene = h1.get(); g_setHW.glass = h2.get(); g_setHW.post = h3.get(); g_setHW.dn = h4.get();
		ok = g_setHW.depth && g_setHW.scene && g_setHW.glass && g_setHW.post && g_setHW.dn;
	}
	if (ok) g_bgHwDone.store(true); else g_bgHwFailed.store(true);
	g_bgState.store(BG_DONE_OK);
}
static void startShaderLoad() {
	if (g_bgThread.joinable()) return;
	g_bgCancel = false;
	g_bgState.store(BG_RUNNING);
	g_bgThread = std::thread(bgLoadWorker);
}
// 毎フレーム、メインスレッドから呼ぶ。完了していたらスレッドを回収し、結果をゲーム側の状態に反映する（PSO はまだ適用しない）
static void pollShaderLoad() {
	// 段階3（HW-RT）が失敗したら、以後は SW-RT（DXBC）だけで動かす。段階3は段階2の後なので SW-RT は既に使える
	if (g_bgHwFailed.load() && !g_bgHwFailApplied) {
		g_bgHwFailApplied = true;
		g_useDXC = g_useDXR = false; g_hwRT = false; g_rtNote = L"HW RT: シェーダーコンパイル失敗 (ソフトウェアBVHで動作)";
	}
	if (!g_bgThread.joinable() || g_bgState.load() == BG_RUNNING) return;
	g_bgThread.join();
	if (g_bgState.load() != BG_DONE_OK) {   // 段階1が失敗した。疑似RT・レベルは使えない
		g_heavyFailed = true;
		std::string e; { std::lock_guard<std::mutex> lk(g_errMtx); e = g_shaderErr; }
		OutputDebugStringA(("バックグラウンドのシェーダー読み込みに失敗しました\n" + e).c_str());
	}
}
// 終了時に呼ぶ。読み込み中なら、コンパイルの完了を待たずにプロセスを終える
static void shutdownShaderLoad() {
	if (g_bgThread.joinable()) {
		g_bgCancel = true;
		if (g_bgState.load() == BG_RUNNING) { gpuIdle(); ExitProcess(0); }
		g_bgThread.join();
	}
	relSet(g_setSW); relSet(g_setHW);
}

static void createTargets() {
	RECT rc; GetClientRect(g_hwnd, &rc);
	g_w = std::max<LONG>(rc.right, 1); g_h = std::max<LONG>(rc.bottom, 1);
	for (UINT i = 0; i < BB_COUNT; i++) { g_sc->GetBuffer(i, IID_PPV_ARGS(&g_bb[i])); g_dev->CreateRenderTargetView(g_bb[i], nullptr, rtvAt(i)); }
	D3D12_HEAP_PROPERTIES hp = heapP(D3D12_HEAP_TYPE_DEFAULT);
	D3D12_SHADER_RESOURCE_VIEW_DESC sv2d{}; sv2d.Format = DXGI_FORMAT_R8G8B8A8_UNORM; sv2d.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
	sv2d.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING; sv2d.Texture2D.MipLevels = 1;

	for (int k = 0; k < 2; k++) {
		D3D12_RESOURCE_DESC d = texDesc(g_w, g_h, DXGI_FORMAT_R8G8B8A8_UNORM, 4, D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET);
		D3D12_CLEAR_VALUE cv{}; cv.Format = DXGI_FORMAT_R8G8B8A8_UNORM; if (k == 0) memcpy(cv.Color, SKY_DAY, sizeof(SKY_DAY));
		ID3D12Resource** pp = (k == 0) ? &g_msColor : &g_msAlb;
		g_dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, D3D12_RESOURCE_STATE_RENDER_TARGET, &cv, IID_PPV_ARGS(pp));
		g_dev->CreateRenderTargetView(*pp, nullptr, rtvAt(BB_COUNT + k));
	}
	{
		D3D12_RESOURCE_DESC d = texDesc(g_w, g_h, DXGI_FORMAT_R32_TYPELESS, 4, D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL);
		D3D12_CLEAR_VALUE cv{}; cv.Format = DXGI_FORMAT_D32_FLOAT; cv.DepthStencil.Depth = 1.0f;
		g_dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, D3D12_RESOURCE_STATE_DEPTH_WRITE, &cv, IID_PPV_ARGS(&g_msDepth));
		D3D12_DEPTH_STENCIL_VIEW_DESC dv{}; dv.Format = DXGI_FORMAT_D32_FLOAT; dv.ViewDimension = D3D12_DSV_DIMENSION_TEXTURE2DMS;
		g_dev->CreateDepthStencilView(g_msDepth, &dv, g_dsvHeap->GetCPUDescriptorHandleForHeapStart());
		D3D12_SHADER_RESOURCE_VIEW_DESC sv{}; sv.Format = DXGI_FORMAT_R32_FLOAT; sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2DMS;
		sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
		g_dev->CreateShaderResourceView(g_msDepth, &sv, srvCpu(4));
		g_dev->CreateShaderResourceView(g_msDepth, &sv, srvCpu(8));
	}

	{
		D3D12_RESOURCE_DESC d = texDesc(g_w, g_h, DXGI_FORMAT_R8G8B8A8_UNORM, 1, D3D12_RESOURCE_FLAG_NONE);
		g_dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, nullptr, IID_PPV_ARGS(&g_resolved));
		g_dev->CreateShaderResourceView(g_resolved, &sv2d, srvCpu(3));
		g_dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, nullptr, IID_PPV_ARGS(&g_resAlb));
		g_dev->CreateShaderResourceView(g_resAlb, &sv2d, srvCpu(6));
		g_dev->CreateShaderResourceView(g_resAlb, &sv2d, srvCpu(10));
		D3D12_RESOURCE_DESC dd = texDesc(g_w, g_h, DXGI_FORMAT_R8G8B8A8_UNORM, 1, D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET);
		g_dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &dd, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, nullptr, IID_PPV_ARGS(&g_dn));
		g_dev->CreateShaderResourceView(g_dn, &sv2d, srvCpu(7));
		g_dev->CreateRenderTargetView(g_dn, nullptr, rtvAt(BB_COUNT + 2));
	}
}
static void releaseTargets() {
	for (UINT i = 0; i < BB_COUNT; i++) if (g_bb[i]) { g_bb[i]->Release(); g_bb[i] = nullptr; }
	ID3D12Resource** all[] = { &g_msColor, &g_msAlb, &g_msDepth, &g_resolved, &g_resAlb, &g_dn };
	for (auto** p : all) if (*p) { (*p)->Release(); *p = nullptr; }
}

static void hudResize() {
	if (!g_dev) return;
	if (g_htex) { g_htex->Release(); g_htex = nullptr; }
	if (g_hudUp) { g_hudUp->Unmap(0, nullptr); g_hudUp->Release(); g_hudUp = nullptr; }
	D3D12_HEAP_PROPERTIES hp = heapP(D3D12_HEAP_TYPE_DEFAULT);
	D3D12_RESOURCE_DESC d = texDesc(g_w, g_h, DXGI_FORMAT_B8G8R8A8_UNORM, 1, D3D12_RESOURCE_FLAG_NONE);
	g_dev->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &d, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, nullptr, IID_PPV_ARGS(&g_htex));
	D3D12_SHADER_RESOURCE_VIEW_DESC sv{}; sv.Format = DXGI_FORMAT_B8G8R8A8_UNORM; sv.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
	sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING; sv.Texture2D.MipLevels = 1;
	g_dev->CreateShaderResourceView(g_htex, &sv, srvCpu(5));
	g_dev->CreateShaderResourceView(g_htex, &sv, srvCpu(9));
	g_hudPitch = ((UINT)g_w * 4 + 255) & ~255u;
	g_hudUp = createBuf((UINT64)g_hudPitch * g_h, D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ);
	void* p = nullptr; g_hudUp->Map(0, nullptr, &p); g_hudUpPtr = (UINT8*)p;

	if (!g_hdc) g_hdc = CreateCompatibleDC(nullptr);
	BITMAPINFO bi{}; bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	bi.bmiHeader.biWidth = g_w; bi.bmiHeader.biHeight = -g_h;
	bi.bmiHeader.biPlanes = 1; bi.bmiHeader.biBitCount = 32; bi.bmiHeader.biCompression = BI_RGB;
	void* bits = nullptr;
	HBITMAP nb = CreateDIBSection(g_hdc, &bi, DIB_RGB_COLORS, &bits, nullptr, 0);
	if (nb && bits) {
		SelectObject(g_hdc, nb);
		if (g_dib) DeleteObject(g_dib);
		g_dib = nb; g_bits = (unsigned char*)bits;
	}
	else if (nb) DeleteObject(nb);

	S = std::min(2.5f, std::max(0.75f, g_h / 720.0f));
	SelectObject(g_hdc, GetStockObject(DEFAULT_GUI_FONT));
	const int base[4] = { 13, 16, 21, 46 }, wt[4] = { 400, 400, 600, 700 };
	for (int i = 0; i < 4; i++) {
		if (g_font[i]) DeleteObject(g_font[i]);
		g_fpx[i] = (int)(base[i] * S + 0.5f);
		g_font[i] = CreateFontW(-g_fpx[i], 0, 0, 0, wt[i], FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS,
			CLIP_DEFAULT_PRECIS, ANTIALIASED_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Meiryo UI");
	}
}
static void onResize() {
	if (!g_sc || !g_dev) return;
	gpuIdle();
	releaseTargets();
	g_sc->ResizeBuffers(0, 0, 0, DXGI_FORMAT_UNKNOWN, 0);
	createTargets();
	hudResize();
}


static void recordTextCopy() {
	for (int y = 0; y < g_h; y++) memcpy(g_hudUpPtr + (size_t)y * g_hudPitch, g_bits + (size_t)y * g_w * 4, (size_t)g_w * 4);
	D3D12_RESOURCE_BARRIER b0 = trans(g_htex, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_COPY_DEST);
	g_cl->ResourceBarrier(1, &b0);
	D3D12_TEXTURE_COPY_LOCATION dst{}, src{};
	dst.pResource = g_htex; dst.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX; dst.SubresourceIndex = 0;
	src.pResource = g_hudUp; src.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;
	src.PlacedFootprint.Footprint.Format = DXGI_FORMAT_B8G8R8A8_UNORM; src.PlacedFootprint.Footprint.Width = g_w;
	src.PlacedFootprint.Footprint.Height = g_h; src.PlacedFootprint.Footprint.Depth = 1; src.PlacedFootprint.Footprint.RowPitch = g_hudPitch;
	g_cl->CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
	D3D12_RESOURCE_BARRIER b1 = trans(g_htex, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
	g_cl->ResourceBarrier(1, &b1);
}


static ID3D12Resource* g_blasW, * g_blasC[3], * g_tlas, * g_tlasScratch, * g_instUp;
static D3D12_RAYTRACING_INSTANCE_DESC* g_instPtr;
static uint32_t g_rtRoots[4] = { 0, 0, 0, 0 }, g_rtTriBase[4] = { 0, 0, 0, 0 };
static const UINT TLAS_INSTANCES = 5;

static ID3D12Resource* buildBLAS(const Mesh& m) {
	D3D12_RAYTRACING_GEOMETRY_DESC gd{};
	gd.Type = D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES; gd.Flags = D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;
	gd.Triangles.VertexBuffer.StartAddress = m.res->GetGPUVirtualAddress() + (UINT64)m.start * sizeof(Vtx);
	gd.Triangles.VertexBuffer.StrideInBytes = sizeof(Vtx);
	gd.Triangles.VertexFormat = DXGI_FORMAT_R32G32B32_FLOAT; gd.Triangles.VertexCount = m.n;
	D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS in{};
	in.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL; in.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_TRACE;
	in.NumDescs = 1; in.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY; in.pGeometryDescs = &gd;
	D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO info{};
	g_dev5->GetRaytracingAccelerationStructurePrebuildInfo(&in, &info);
	ID3D12Resource* scratch = createBuf((info.ScratchDataSizeInBytes + 255) & ~255ull, D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
	ID3D12Resource* result = createBuf((info.ResultDataMaxSizeInBytes + 255) & ~255ull, D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
	if (!scratch || !result) return nullptr;
	D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC bd{};
	bd.Inputs = in; bd.DestAccelerationStructureData = result->GetGPUVirtualAddress(); bd.ScratchAccelerationStructureData = scratch->GetGPUVirtualAddress();
	g_cl4->BuildRaytracingAccelerationStructure(&bd, 0, nullptr);
	D3D12_RESOURCE_BARRIER ub = uavB(result); g_cl->ResourceBarrier(1, &ub);
	g_pending.push_back(scratch);
	return result;
}


static bool initRTScene(const Verts& world, const Verts car[3], const Mesh& worldMesh, const Mesh carMesh[3]) {
	std::vector<float> tris; std::vector<BNode> nodes;
	const Verts* src[4] = { &world, &car[0], &car[1], &car[2] };
	if (g_useDXR) {
		for (int k = 0; k < 4; k++) { g_rtTriBase[k] = (uint32_t)(tris.size() / 12); appendTrisOnly(tris, *src[k]); }
	}
	else {
		std::vector<BNode> ln[4]; std::vector<float> lt[4];
		std::vector<std::future<void>> fs;
		for (int k = 0; k < 4; k++) fs.push_back(std::async(std::launch::async, [&, k]() { buildBVHLocal(*src[k], ln[k], lt[k]); }));
		for (auto& f : fs) f.get();
		for (int k = 0; k < 4; k++) { g_rtTriBase[k] = (uint32_t)(tris.size() / 12); g_rtRoots[k] = mergeBVH(nodes, tris, ln[k], lt[k]); }
	}
	ID3D12Resource* nodeBuf = makeStaticBuffer(g_useDXR ? nullptr : nodes.data(), g_useDXR ? 32 : nodes.size() * sizeof(BNode));
	ID3D12Resource* triBuf = makeStaticBuffer(tris.data(), tris.size() * sizeof(float));
	if (!nodeBuf || !triBuf) return false;
	D3D12_SHADER_RESOURCE_VIEW_DESC sv{}; sv.Format = DXGI_FORMAT_UNKNOWN; sv.ViewDimension = D3D12_SRV_DIMENSION_BUFFER;
	sv.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING; sv.Buffer.StructureByteStride = 16;
	sv.Buffer.NumElements = (UINT)((g_useDXR ? 32 : nodes.size() * sizeof(BNode)) / 16);
	g_dev->CreateShaderResourceView(nodeBuf, &sv, srvCpu(0));
	sv.Buffer.NumElements = (UINT)(tris.size() * sizeof(float) / 16);
	g_dev->CreateShaderResourceView(triBuf, &sv, srvCpu(1));

	if (g_useDXR) {
		g_blasW = buildBLAS(worldMesh);
		for (int i = 0; i < 3; i++) g_blasC[i] = buildBLAS(carMesh[i]);
		D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_INPUTS in{};
		in.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL; in.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_BUILD;
		in.NumDescs = TLAS_INSTANCES; in.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY;
		D3D12_RAYTRACING_ACCELERATION_STRUCTURE_PREBUILD_INFO info{};
		g_dev5->GetRaytracingAccelerationStructurePrebuildInfo(&in, &info);
		g_tlas = createBuf((info.ResultDataMaxSizeInBytes + 255) & ~255ull, D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
		g_tlasScratch = createBuf((info.ScratchDataSizeInBytes + 255) & ~255ull, D3D12_HEAP_TYPE_DEFAULT, D3D12_RESOURCE_STATE_UNORDERED_ACCESS, D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS);
		g_instUp = createBuf(TLAS_INSTANCES * sizeof(D3D12_RAYTRACING_INSTANCE_DESC), D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ);
		if (!g_blasW || !g_blasC[0] || !g_blasC[1] || !g_blasC[2] || !g_tlas || !g_tlasScratch || !g_instUp) return false;
		void* p = nullptr; g_instUp->Map(0, nullptr, &p); g_instPtr = (D3D12_RAYTRACING_INSTANCE_DESC*)p;
		D3D12_SHADER_RESOURCE_VIEW_DESC av{}; av.ViewDimension = D3D12_SRV_DIMENSION_RAYTRACING_ACCELERATION_STRUCTURE;
		av.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING; av.RaytracingAccelerationStructure.Location = g_tlas->GetGPUVirtualAddress();
		g_dev->CreateShaderResourceView(nullptr, &av, srvCpu(2));
	}
	else {
		D3D12_SHADER_RESOURCE_VIEW_DESC nd{}; nd.Format = DXGI_FORMAT_R8G8B8A8_UNORM; nd.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
		nd.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING; nd.Texture2D.MipLevels = 1;
		g_dev->CreateShaderResourceView(nullptr, &nd, srvCpu(2));
	}
	return true;
}


static void updateTLAS(const M4* cw, const uint32_t* types, int nCars) {
	auto setInst = [&](UINT k, const M4* m, UINT id, ID3D12Resource* blas) {
		D3D12_RAYTRACING_INSTANCE_DESC& x = g_instPtr[k]; memset(&x, 0, sizeof(x));
		if (m) { for (int i = 0; i < 3; i++) for (int j = 0; j < 4; j++) x.Transform[i][j] = m->m[j][i]; }
		else { x.Transform[0][0] = x.Transform[1][1] = x.Transform[2][2] = 1.0f; }
		x.InstanceID = id; x.InstanceMask = 0xFF; x.InstanceContributionToHitGroupIndex = 0; x.Flags = 0;
		x.AccelerationStructure = blas->GetGPUVirtualAddress();
		};
	setInst(0, nullptr, 0, g_blasW);
	for (int i = 0; i < nCars; i++) { UINT type = types[i]; setInst(1 + i, &cw[i], type, g_blasC[type - 1]); }
	D3D12_BUILD_RAYTRACING_ACCELERATION_STRUCTURE_DESC bd{};
	bd.Inputs.Type = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL; bd.Inputs.Flags = D3D12_RAYTRACING_ACCELERATION_STRUCTURE_BUILD_FLAG_PREFER_FAST_BUILD;
	bd.Inputs.NumDescs = TLAS_INSTANCES; bd.Inputs.DescsLayout = D3D12_ELEMENTS_LAYOUT_ARRAY; bd.Inputs.InstanceDescs = g_instUp->GetGPUVirtualAddress();
	bd.DestAccelerationStructureData = g_tlas->GetGPUVirtualAddress(); bd.ScratchAccelerationStructureData = g_tlasScratch->GetGPUVirtualAddress();
	g_cl4->BuildRaytracingAccelerationStructure(&bd, 0, nullptr);
	D3D12_RESOURCE_BARRIER ub = uavB(g_tlas); g_cl->ResourceBarrier(1, &ub);
}


static bool initD3D(HWND hwnd) {
	g_hwnd = hwnd;
	IDXGIFactory4* fac = nullptr;
	if (FAILED(CreateDXGIFactory2(0, IID_PPV_ARGS(&fac)))) return false;
	auto tryAdapter = [&](IDXGIAdapter1* a) {
		DXGI_ADAPTER_DESC1 ad; a->GetDesc1(&ad);
		if (ad.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) return false;
		if (FAILED(D3D12CreateDevice(a, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&g_dev)))) return false;
		g_gpuName = ad.Description; return true;
		};
	IDXGIFactory6* f6 = nullptr;
	if (SUCCEEDED(fac->QueryInterface(IID_PPV_ARGS(&f6)))) {
		for (UINT i = 0; !g_dev; i++) {
			IDXGIAdapter1* a = nullptr;
			if (f6->EnumAdapterByGpuPreference(i, DXGI_GPU_PREFERENCE_HIGH_PERFORMANCE, IID_PPV_ARGS(&a)) == DXGI_ERROR_NOT_FOUND) break;
			tryAdapter(a); a->Release();
		}
		f6->Release();
	}
	for (UINT i = 0; !g_dev; i++) {
		IDXGIAdapter1* a = nullptr;
		if (fac->EnumAdapters1(i, &a) == DXGI_ERROR_NOT_FOUND) break;
		tryAdapter(a); a->Release();
	}
	if (!g_dev) {
		IDXGIAdapter* warp = nullptr;
		if (SUCCEEDED(fac->EnumWarpAdapter(IID_PPV_ARGS(&warp)))) { if (SUCCEEDED(D3D12CreateDevice(warp, D3D_FEATURE_LEVEL_11_0, IID_PPV_ARGS(&g_dev)))) { g_gpuName = L"WARP (ソフトウェア)"; g_swGpu = true; } warp->Release(); }
	}
	if (!g_dev) return false;


	bool tier11 = false, sm65 = false;
	{ D3D12_FEATURE_DATA_D3D12_OPTIONS5 o5{}; tier11 = SUCCEEDED(g_dev->CheckFeatureSupport(D3D12_FEATURE_D3D12_OPTIONS5, &o5, sizeof(o5))) && o5.RaytracingTier >= D3D12_RAYTRACING_TIER_1_1; }
	{ D3D12_FEATURE_DATA_SHADER_MODEL sm{ D3D_SHADER_MODEL_6_5 }; sm65 = SUCCEEDED(g_dev->CheckFeatureSupport(D3D12_FEATURE_SHADER_MODEL, &sm, sizeof(sm))) && sm.HighestShaderModel >= D3D_SHADER_MODEL_6_5; }
	g_dev->QueryInterface(IID_PPV_ARGS(&g_dev5));

	D3D12_COMMAND_QUEUE_DESC qd{}; qd.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
	if (FAILED(g_dev->CreateCommandQueue(&qd, IID_PPV_ARGS(&g_q)))) return false;
	g_dev->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&g_alloc));
	g_dev->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, g_alloc, nullptr, IID_PPV_ARGS(&g_cl));
	g_cl->QueryInterface(IID_PPV_ARGS(&g_cl4));
	g_dev->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&g_fence));
	g_fenceEvt = CreateEventW(nullptr, FALSE, FALSE, nullptr);


	bool wantDXR = tier11 && sm65 && g_dev5 && g_cl4;
	if (!wantDXR) g_rtNote = tier11 ? L"HW RT: 無効 (シェーダーモデル6.5未対応)" : L"HW RT: 非搭載 (ソフトウェアBVHで動作)";
	else if (!loadDXC()) { wantDXR = false; g_rtNote = L"HW RT: dxcompiler.dll が見つかりません (ソフトウェアBVHで動作)"; }
	g_useDXC = wantDXR; g_useDXR = wantDXR;
	if (!compileLite()) {   // 重いシェーダーのコンパイルは、グラフィックレベルが「最低」以降になった時まで行わない
		if (wantDXR) {
			MessageBoxA(nullptr, ("ハードウェアRT用シェーダーのコンパイルに失敗したため、ソフトウェアRTに切り替えます。\n\n" + g_shaderErr.substr(0, 1800)).c_str(), "TrainSim", MB_OK | MB_ICONWARNING);
			g_shaderErr.clear(); g_useDXC = g_useDXR = false; g_rtNote = L"HW RT: シェーダーコンパイル失敗 (ソフトウェアBVHで動作)";
			if (!compileLite()) return false;
		}
		else return false;
	}
	if (g_useDXR) g_rtNote = L"HW RT: 有効 (DXR 1.1)";
	g_hwRT = g_useDXR;


	{
		RECT rc; GetClientRect(hwnd, &rc);
		DXGI_SWAP_CHAIN_DESC1 sd{}; sd.Width = std::max<LONG>(rc.right, 1); sd.Height = std::max<LONG>(rc.bottom, 1); sd.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
		sd.SampleDesc.Count = 1; sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT; sd.BufferCount = BB_COUNT; sd.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
		IDXGISwapChain1* sc1 = nullptr;
		if (FAILED(fac->CreateSwapChainForHwnd(g_q, hwnd, &sd, nullptr, nullptr, &sc1))) return false;
		sc1->QueryInterface(IID_PPV_ARGS(&g_sc)); sc1->Release();
		fac->MakeWindowAssociation(hwnd, DXGI_MWA_NO_ALT_ENTER);
	}
	fac->Release();

	D3D12_DESCRIPTOR_HEAP_DESC hd{}; hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV; hd.NumDescriptors = BB_COUNT + 3;
	g_dev->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&g_rtvHeap));
	hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV; hd.NumDescriptors = 1; g_dev->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&g_dsvHeap));
	hd.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV; hd.NumDescriptors = 11; hd.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
	g_dev->CreateDescriptorHeap(&hd, IID_PPV_ARGS(&g_srvHeap));
	g_rtvInc = g_dev->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_RTV);
	g_srvInc = g_dev->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

	g_rsScene = makeRS(true); g_rsPost = makeRS(false);
	if (!g_rsScene || !g_rsPost) return false;
	if (!createLitePSOs()) return false;   // 「最低」以降用の PSO は、起動後にバックグラウンドで作る (startShaderLoad)

	g_cbUp = createBuf(65536, D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ);
	{ void* p = nullptr; g_cbUp->Map(0, nullptr, &p); g_cbPtr = (UINT8*)p; }

	createTargets();
	hudResize();
	return true;
}


struct Col { float r, g, b, a; };
struct HV { float x, y; Col c; };
struct TItem { std::wstring s; int x, y, font; COLORREF col; UINT align; };
static std::vector<HV> g_hv;
static std::vector<TItem> g_tx;
static float g_menuR[11][4] = {};
static float g_respR[4] = {};
static bool  g_respHover = false;

static void hQuad(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, Col c) {
	HV q[4] = { { x0,y0,c }, { x1,y1,c }, { x2,y2,c }, { x3,y3,c } };
	const int idx[6] = { 0, 1, 2, 0, 2, 3 };
	for (int i : idx) g_hv.push_back(q[i]);
}
static void hRect(float x0, float y0, float x1, float y1, Col c) { hQuad(x0, y0, x1, y0, x1, y1, x0, y1, c); }
static void hLine(float x0, float y0, float x1, float y1, float th, Col c) {
	float dx = x1 - x0, dy = y1 - y0, l = sqrtf(dx * dx + dy * dy);
	if (l < 1e-4f) return;
	float nx = -dy / l * th * 0.5f, ny = dx / l * th * 0.5f;
	hQuad(x0 + nx, y0 + ny, x1 + nx, y1 + ny, x1 - nx, y1 - ny, x0 - nx, y0 - ny, c);
}
static void hText(const std::wstring& s, float x, float y, int font, COLORREF col, UINT align) {
	g_tx.push_back({ s, (int)(x + 0.5f), (int)(y + 0.5f), font, col, align });
}

static void hTextO(const std::wstring& s, float x, float y, int font, COLORREF col, UINT align) {
	const float o = std::max(1.0f, 1.5f * S);
	const float dx[8] = { -o, o, 0, 0, -o, o, -o, o }, dy[8] = { 0, 0, -o, o, -o, -o, o, o };
	for (int k = 0; k < 8; k++) hText(s, x + dx[k], y + dy[k], font, RGB(0, 0, 0), align);
	hText(s, x, y, font, col, align);
}

static void hDisc(float cx, float cy, float r, Col c, int seg = 20) {
	for (int i = 0; i < seg; i++) {
		const float a0 = 2.0f * PI * i / seg, a1 = 2.0f * PI * (i + 1) / seg;
		const float x0 = cx + r * cosf(a0), y0 = cy + r * sinf(a0), x1 = cx + r * cosf(a1), y1 = cy + r * sinf(a1);
		hQuad(cx, cy, x0, y0, x1, y1, x1, y1, c);
	}
}

struct HudInfo {
	float kmh = 0;
	int handle = 0;
	float d = 0;
	std::wstring nextStationName;
	float doorPosLeft = 0;
	float doorPosRight = 0;
	bool doorTargetLeft = false;
	bool doorTargetRight = false;
	const wchar_t* cam = L"";
	std::wstring toast; float toastT = 0;
	std::wstring result;
	bool pause = false;
	int rev = 1;
	float tod = 15.0f; bool todFrozen = false;
	int menuSel = 0;
	int gfx = 0; bool hwRT = false, swGpu = false; bool pseudo = false; bool pseudoUnlocked = false, rtUnlocked = false; std::wstring gpu, rtNote;
	bool shaderLoading = false, shaderFailed = false;   // シェーダーをバックグラウンドで読み込み中 / 読み込みに失敗
	bool mouseMode = false;
	bool cabView = false;
	bool driving = false;
	int hp = 1000;
	bool dead = false;
	float hurt = 0.0f;
	int ato = 0;
	bool cross = false;
	bool crossHot = false;
	std::wstring prompt;
	std::vector<std::wstring> info; float infoT = 0.0f;
};

static void buildHud(const HudInfo& in) {
	g_hv.clear(); g_tx.clear();
	const float W = (float)g_w, H = (float)g_h;
	const Col panel = { 0.02f, 0.03f, 0.06f, 0.62f };
	const Col white = { 1, 1, 1, 1 };
	const COLORREF cWhite = RGB(255, 255, 255), cGray = RGB(170, 170, 170);
	wchar_t buf[64];

	if (in.hurt > 0.0f) hRect(0, 0, W, H, Col{ 0.85f, 0.0f, 0.0f, 0.38f * std::min(1.0f, in.hurt) });

	if (in.driving) {
		hRect(16 * S, 16 * S, 300 * S, 46 * S, panel);
		swprintf_s(buf, L"カメラ: %s", in.cam);
		hText(buf, 28 * S, 22 * S, 1, cWhite, TA_LEFT);
	}

	{
		const float x0 = 16 * S, y0 = (in.driving ? 52 : 16) * S, pw = 284 * S, ph = 40 * S;
		hRect(x0, y0, x0 + pw, y0 + ph, panel);
		const int hpv = std::max(0, std::min(1000, in.hp));
		const float fr = hpv / 1000.0f;
		swprintf_s(buf, L"体力  %d / 1000", hpv);
		hText(buf, x0 + 12 * S, y0 + 3 * S, 0, cWhite, TA_LEFT);
		const float bx0 = x0 + 12 * S, bx1 = x0 + pw - 12 * S, by = y0 + 24 * S, bh = 9 * S;
		hRect(bx0, by, bx1, by + bh, Col{ 0.4f, 0.4f, 0.45f, 0.5f });
		const Col fc = fr > 0.5f ? Col{ 0.30f, 0.90f, 0.40f, 1 } : (fr > 0.25f ? Col{ 1.0f, 0.80f, 0.20f, 1 } : Col{ 1.0f, 0.25f, 0.20f, 1 });
		if (fr > 0.0f) hRect(bx0, by, bx0 + (bx1 - bx0) * fr, by + bh, fc);
	}

	{
		const float x0 = 16 * S, y0 = (in.driving ? 132 : 62) * S, pw = 284 * S, ph = 28 * S;
		hRect(x0, y0, x0 + pw, y0 + ph, panel);
		const int mins = ((int)(in.tod * 60.0f + 0.5f)) % 1440;
		swprintf_s(buf, L"時刻 %02d:%02d%s", mins / 60, mins % 60, in.todFrozen ? L"   （フリーズ中）" : L"");
		hText(buf, x0 + 12 * S, y0 + 5 * S, 0, in.todFrozen ? RGB(255, 190, 60) : cWhite, TA_LEFT);
	}

	if (in.driving) {
		const float x0 = 16 * S, y0 = 98 * S, pw = 284 * S, ph = 28 * S;
		hRect(x0, y0, x0 + pw, y0 + ph, panel);
		const wchar_t* st = in.ato == 1 ? L"自動運転中（Bで非常ブレーキ）" : (in.ato == 2 ? L"待機中（Rでノッチをリセットして再開）" : L"手動運転");
		swprintf_s(buf, L"ATO[Q]: %s", st);
		hText(buf, x0 + 12 * S, y0 + 5 * S, 0, in.ato == 1 ? RGB(90, 255, 130) : (in.ato == 2 ? RGB(255, 190, 60) : cGray), TA_LEFT);
	}

	if (in.driving) {
		if (in.cabView) {
			const float PW = 170 * S, PH = 96 * S, x0 = 16 * S, y0 = H - PH - 16 * S;
			hRect(x0, y0, x0 + PW, y0 + PH, panel);
			swprintf_s(buf, L"%d", (int)(in.kmh + 0.5f));
			hText(buf, x0 + PW / 2, y0 + 14 * S, 3, cWhite, TA_CENTER);
			hText(L"km/h", x0 + PW / 2, y0 + 14 * S + g_fpx[3] * 1.25f, 0, cGray, TA_CENTER);
		}
		else {
			const float D = 260 * S, x0 = 16 * S, y0 = H - D - 16 * S;
			hRect(x0, y0, x0 + D, y0 + D, panel);
			const float cx = x0 + D / 2, cy = y0 + D / 2 - 8 * S, r = D * 0.40f;
			const float maxKmh = 120.0f;
			const float frac = std::min(1.0f, in.kmh / maxKmh);
			auto ang = [](float f) { return (225.0f - 270.0f * f) * PI / 180.0f; };
			for (int i = 0; i < 90; i++) {
				float f0 = i / 90.0f, f1 = (i + 1) / 90.0f, a0 = ang(f0), a1 = ang(f1);
				Col c = (f1 <= frac + 1e-4f) ? Col{ 0.30f, 0.85f, 1.0f, 1.0f } : Col{ 0.45f, 0.47f, 0.55f, 0.6f };
				hLine(cx + r * cosf(a0), cy - r * sinf(a0), cx + r * cosf(a1), cy - r * sinf(a1), 4 * S, c);
			}
			for (int k = 0; k <= 12; k++) {
				float a = ang(k / 12.0f), ca = cosf(a), sa = sinf(a);
				bool major = (k % 2 == 0);
				float r0 = r * (major ? 0.78f : 0.86f), r1 = r * 0.93f;
				hLine(cx + r0 * ca, cy - r0 * sa, cx + r1 * ca, cy - r1 * sa, (major ? 2.5f : 1.5f) * S, white);
				if (major) {
					swprintf_s(buf, L"%d", k * 10);
					hText(buf, cx + r * 0.62f * ca, cy - r * 0.62f * sa - g_fpx[0] * 0.6f, 0, cWhite, TA_CENTER);
				}
			}
			float a = ang(frac);
			hLine(cx - r * 0.12f * cosf(a), cy + r * 0.12f * sinf(a), cx + r * 0.82f * cosf(a), cy - r * 0.82f * sinf(a), 4 * S, Col{ 1.0f, 0.35f, 0.30f, 1.0f });
			hRect(cx - 5 * S, cy - 5 * S, cx + 5 * S, cy + 5 * S, white);
			swprintf_s(buf, L"%d", (int)(in.kmh + 0.5f));
			hText(buf, cx, cy + r * 0.30f, 3, cWhite, TA_CENTER);
			hText(L"km/h", cx, cy + r * 0.30f + g_fpx[3] * 1.25f, 0, cGray, TA_CENTER);
		}
	}

	if (in.driving) {
		const float pw = 130 * S, rh = 19 * S, ph = (30 + 15 * 19 + 10) * S;
		const float x1 = W - 16 * S, x0 = x1 - pw, y1 = H - 16 * S, y0 = y1 - ph;
		hRect(x0, y0, x1, y1, panel);
		hText(L"ノッチ", (x0 + x1) / 2, y0 + 6 * S, 1, cWhite, TA_CENTER);
		for (int j = 0; j < 15; j++) {
			int hd = 5 - j;
			float y = y0 + 30 * S + j * rh;
			Col base; wchar_t lb[8];
			if (hd > 0) { base = { 0.30f, 1.00f, 0.45f, 1 }; swprintf_s(lb, L"P%d", hd); }
			else if (hd == 0) { base = { 1.00f, 1.00f, 1.00f, 1 }; wcscpy_s(lb, L"N"); }
			else if (hd > -9) { base = { 1.00f, 0.70f, 0.20f, 1 }; swprintf_s(lb, L"B%d", -hd); }
			else { base = { 1.00f, 0.25f, 0.20f, 1 }; swprintf_s(lb, L"EB"); }
			bool cur = (hd == in.handle);
			base.a = cur ? 0.95f : 0.18f;
			hRect(x0 + 46 * S, y + 2 * S, x1 - 10 * S, y + rh - 2 * S, base);
			hText(lb, x0 + 14 * S, y + 1 * S, 0, cur ? cWhite : cGray, TA_LEFT);
		}
	}

	if (in.driving) {
		const float pw = 100 * S, rh = 19 * S, ph = (30 + 3 * 19 + 10) * S;
		const float x1 = W - 16 * S - 130 * S - 10 * S, x0 = x1 - pw, y1 = H - 16 * S, y0 = y1 - ph;
		hRect(x0, y0, x1, y1, panel);
		hText(L"レバーサー", (x0 + x1) / 2, y0 + 6 * S, 1, cWhite, TA_CENTER);
		const wchar_t* nm[3] = { L"前", L"切", L"後" };
		const int val[3] = { 1, 0, -1 };
		const Col cc[3] = { { 0.30f, 1.00f, 0.45f, 1 }, { 1.00f, 1.00f, 1.00f, 1 }, { 1.00f, 0.70f, 0.20f, 1 } };
		for (int j = 0; j < 3; j++) {
			float y = y0 + 30 * S + j * rh;
			bool cur = (val[j] == in.rev);
			Col base = cc[j]; base.a = cur ? 0.95f : 0.18f;
			hRect(x0 + 46 * S, y + 2 * S, x1 - 10 * S, y + rh - 2 * S, base);
			hText(nm[j], x0 + 14 * S, y + 1 * S, 0, cur ? cWhite : cGray, TA_LEFT);
		}
	}

	if (in.driving) {
		const float sw = 500 * S, sh = 130 * S, x0 = W / 2 - sw / 2, y0 = 16 * S, x1 = x0 + sw;
		hRect(x0, y0, x1, y0 + sh, panel);
		swprintf_s(buf, L"次駅: %s (%s)", in.nextStationName.c_str(), in.d > 200.0f ? L"停止位置まで" : L"停止位置");
		hText(buf, x0 + 14 * S, y0 + 9 * S, 1, cWhite, TA_LEFT);
		const bool over = in.d < 0;
		{
			float a = fabsf(in.d); wchar_t num[32];
			if (a < 1.0f) swprintf_s(num, L"%d cm", (int)(a * 100.0f + 0.5f)); else swprintf_s(num, L"%.1f m", a);
			if (over) swprintf_s(buf, L"オーバーラン %s", num); else wcscpy_s(buf, num);
		}
		COLORREF vc = over ? RGB(255, 80, 70) : (in.d < 30 ? RGB(255, 230, 60) : RGB(90, 220, 255));
		hText(buf, x1 - 14 * S, y0 + 5 * S, 2, vc, TA_RIGHT);

		const float bx0 = x0 + 14 * S, bx1 = x1 - 14 * S, by = y0 + 44 * S, bh = 10 * S;
		hRect(bx0, by, bx1, by + bh, Col{ 0.4f, 0.4f, 0.45f, 0.5f });
		float f = over ? 1.0f : std::min(1.0f, std::max(0.0f, 1.0f - in.d / 200.0f));
		Col fc = (fabsf(in.d) <= 1.0f) ? Col{ 0.30f, 0.90f, 0.40f, 1 } : (over ? Col{ 1.0f, 0.30f, 0.25f, 1 } : (in.d < 30 ? Col{ 1.0f, 0.88f, 0.25f, 1 } : Col{ 0.35f, 0.85f, 1.0f, 1 }));
		hRect(bx0, by, bx0 + (bx1 - bx0) * f, by + bh, fc);
		hRect(bx1 - 2 * S, by - 6 * S, bx1 + 1 * S, by + bh + 6 * S, white);


		const float ry = y0 + 64 * S;
		hText(L"左[O]:", x0 + 14 * S, ry, 0, cWhite, TA_LEFT);
		const wchar_t* dsL = (in.doorPosLeft >= 1.0f) ? L"開" : ((in.doorPosLeft <= 0.0f) ? L"閉" : L"動作中");
		COLORREF dcL = (in.doorPosLeft >= 1.0f) ? RGB(90, 255, 130) : RGB(255, 255, 255);
		hText(dsL, x0 + 65 * S, ry, 0, dcL, TA_LEFT);

		hText(L"右[P]:", x0 + 130 * S, ry, 0, cWhite, TA_LEFT);
		const wchar_t* dsR = (in.doorPosRight >= 1.0f) ? L"開" : ((in.doorPosRight <= 0.0f) ? L"閉" : L"動作中");
		COLORREF dcR = (in.doorPosRight >= 1.0f) ? RGB(90, 255, 130) : RGB(255, 255, 255);
		hText(dsR, x0 + 180 * S, ry, 0, dcR, TA_LEFT);

		const float ry2 = y0 + 96 * S;
		if (!in.result.empty()) {
			COLORREF resCol = (in.result == L"Great!") ? RGB(90, 255, 130) : RGB(255, 230, 60);
			hText(in.result, x1 - 14 * S, ry2, 1, resCol, TA_RIGHT);
		}
	}

	{
		if (!in.info.empty() && in.infoT > 0.0f) {
			const float x0 = 16 * S, y0 = 96 * S, pw = 420 * S, lh = 22 * S, ph = lh * (float)in.info.size() + 10 * S;
			hRect(x0, y0, x0 + pw, y0 + ph, panel);
			for (size_t k = 0; k < in.info.size(); k++) hText(in.info[k], x0 + 12 * S, y0 + 6 * S + lh * (float)k, 0, k == 0 ? RGB(255, 220, 90) : cWhite, TA_LEFT);
		}
	}

	if (in.toastT > 0 && !in.toast.empty()) {
		float tw = std::max(440.0f * S, (float)in.toast.size() * 17.0f * S + 40.0f * S), th = 34 * S, x0 = W / 2 - tw / 2, y0 = H - 16 * S - th;
		hRect(x0, y0, x0 + tw, y0 + th, Col{ 0.40f, 0.06f, 0.06f, 0.85f });
		hText(in.toast, W / 2, y0 + 7 * S, 1, cWhite, TA_CENTER);
	}

	if (in.cross) {
		const float cx = W * 0.5f, cy = H * 0.5f;
		hDisc(cx, cy, 5.4f * S, Col{ 0, 0, 0, 0.80f });
		hDisc(cx, cy, 3.4f * S, Col{ 1, 1, 1, 1 });
		if (in.crossHot) {
			const int N = 28; const float R = 12.0f * S;
			for (int i = 0; i < N; i++) {
				const float a0 = 2.0f * PI * i / N, a1 = 2.0f * PI * (i + 1) / N;
				hLine(cx + R * cosf(a0), cy + R * sinf(a0), cx + R * cosf(a1), cy + R * sinf(a1), 5.5f * S, Col{ 0, 0, 0, 0.80f });
			}
			for (int i = 0; i < N; i++) {
				const float a0 = 2.0f * PI * i / N, a1 = 2.0f * PI * (i + 1) / N;
				hLine(cx + R * cosf(a0), cy + R * sinf(a0), cx + R * cosf(a1), cy + R * sinf(a1), 2.6f * S, Col{ 1, 1, 1, 1 });
			}
		}
	}
	if (!in.prompt.empty() && !in.dead) hTextO(in.prompt, W * 0.5f, in.cross ? H * 0.5f + 24 * S : H - 84 * S, 1, RGB(255, 255, 255), TA_CENTER);

	if (in.dead) {
		hRect(0, 0, W, H, Col{ 0.35f, 0.0f, 0.0f, 0.50f });
		const float top = H * 0.5f - 130 * S, bot = H * 0.5f + 130 * S;
		hRect(0, top, W, bot, Col{ 0.0f, 0.0f, 0.0f, 0.62f });
		hLine(0, top, W, top, 3 * S, Col{ 0.90f, 0.12f, 0.10f, 0.95f });
		hLine(0, bot, W, bot, 3 * S, Col{ 0.90f, 0.12f, 0.10f, 0.95f });
		hTextO(L"死んでしまった", W * 0.5f, top + 30 * S, 3, RGB(255, 70, 60), TA_CENTER);
		const float sy = top + 30 * S + g_fpx[3] * 1.45f;
		hTextO(L"死因:人身事故", W * 0.5f, sy, 2, RGB(255, 255, 255), TA_CENTER);
		const float bw = 250 * S, bh = 50 * S, bx0 = W * 0.5f - bw * 0.5f, by0 = sy + g_fpx[2] * 2.2f, bx1 = bx0 + bw, by1 = by0 + bh;
		g_respR[0] = bx0; g_respR[1] = by0; g_respR[2] = bx1; g_respR[3] = by1;
		hRect(bx0, by0, bx1, by1, g_respHover ? Col{ 0.95f, 0.30f, 0.25f, 0.95f } : Col{ 0.60f, 0.12f, 0.10f, 0.90f });
		const Col bc = { 1, 1, 1, 0.9f };
		hLine(bx0, by0, bx1, by0, 2 * S, bc); hLine(bx0, by1, bx1, by1, 2 * S, bc);
		hLine(bx0, by0, bx0, by1, 2 * S, bc); hLine(bx1, by0, bx1, by1, 2 * S, bc);
		hTextO(L"リスポーン", W * 0.5f, by0 + (bh - g_fpx[2] * 1.25f) * 0.5f, 2, RGB(255, 255, 255), TA_CENTER);
		hText(L"Enterキーでもリスポーンできます", W * 0.5f, by1 + 8 * S, 0, cGray, TA_CENTER);
	}

	if (in.pause) {
		float pw = 520 * S, ph = 572 * S, x0 = W / 2 - pw / 2, y0 = H / 2 - ph / 2;
		hRect(x0, y0, x0 + pw, y0 + ph, Col{ 0.02f, 0.03f, 0.06f, 0.85f });
		hText(L"一時停止中", W / 2, y0 + 12 * S, 2, cWhite, TA_CENTER);
		hText(L"SilkHat 3D WalkingSimulator Ver.26.11.24", W / 2, y0 + 44 * S, 0, RGB(65, 105, 225), TA_CENTER);
		for (int k = 0; k < 2; k++) {
			float bx0 = x0 + 90 * S, bx1 = x0 + pw - 90 * S, by0 = y0 + 76 * S + k * 36 * S, by1 = by0 + 30 * S;
			g_menuR[k][0] = bx0; g_menuR[k][1] = by0; g_menuR[k][2] = bx1; g_menuR[k][3] = by1;
			bool sel = (in.menuSel == k);
			hRect(bx0, by0, bx1, by1, sel ? Col{ 0.30f, 0.65f, 1.0f, 0.55f } : Col{ 0.40f, 0.40f, 0.45f, 0.25f });
			if (k == 0) {
				const int mins = ((int)(in.tod * 60.0f + 0.5f)) % 1440;
				swprintf_s(buf, L"◀   時刻の設定  %02d:%02d   ▶", mins / 60, mins % 60);
			}
			else swprintf_s(buf, L"%s 時刻のフリーズ: %s", in.todFrozen ? L"●" : L"○", in.todFrozen ? L"ON（時刻が止まっています）" : L"OFF（時刻が進みます）");
			hText(buf, W / 2, by0 + 5 * S, 1, cWhite, TA_CENTER);
		}
		hText(L"グラフィックレベル（レイトレーシングの強さ）", W / 2, y0 + 150 * S, 0, cGray, TA_CENTER);
		for (int j = 0; j < 7; j++) {
			const int k = 2 + j;
			const bool dis = (j >= 1 && in.shaderFailed) || (j == 4 && in.swGpu) || (j >= 5 && !in.hwRT);
			const bool locked = (j >= 1 && !in.rtUnlocked);   // RT用の読み込みが終わるまで、「最低」以降は選べない
			const bool off = dis || locked;
			float bx0 = x0 + 90 * S, bx1 = x0 + pw - 90 * S, by0 = y0 + 172 * S + j * 31 * S + (j >= 1 ? 22 * S : 0.0f), by1 = by0 + 27 * S;   // 「オフ」の下に注意書きの行ぶん空ける
			g_menuR[k][0] = bx0; g_menuR[k][1] = by0; g_menuR[k][2] = bx1; g_menuR[k][3] = by1;
			bool sel = (in.menuSel == k) && !off, active = (in.gfx == j) && !in.pseudo;
			hRect(bx0, by0, bx1, by1, off ? Col{ 0.25f, 0.25f, 0.27f, 0.15f } : sel ? Col{ 0.30f, 0.65f, 1.0f, 0.55f } : Col{ 0.40f, 0.40f, 0.45f, 0.25f });
			if (j >= 1 && in.shaderFailed) swprintf_s(buf, L"× %s（シェーダーの読み込みに失敗したため選択不可）", GFX_NAME[j]);
			else if (dis) swprintf_s(buf, (j == 4) ? L"× %s（Direct3D 12 非対応GPUのため選択不可）" : L"× %s（RTコア非搭載のため選択不可）", GFX_NAME[j]);
			else if (locked) swprintf_s(buf, L"× %s（読み込み中のため選択不可）", GFX_NAME[j]);
			else swprintf_s(buf, L"%s %s%s", active ? L"●" : L"○", GFX_NAME[j], j == 0 ? L"（既定・シェーダー処理なし）" : L"");
			hText(buf, W / 2, by0 + (off ? 6 : 4) * S, off ? 0 : 1, off ? RGB(110, 110, 110) : cWhite, TA_CENTER);
			if (j == 0) hText(in.shaderFailed ? L"シェーダーの読み込みに失敗しました" : !in.pseudoUnlocked ? L"疑似レイトレーシング用のシェーダーを読み込み中です" : !in.rtUnlocked ? L"RT用のシェーダーを読み込み中です。完了すると「最低」以降を選べます" : L"レイトレーシングをオンにすると、読み込み中に短時間フリーズすることがあります。", W / 2, by1 + 4 * S, 0, RGB(255, 200, 90), TA_CENTER);
		}
		hText(L"視点操作（追従・一人称視点）", W / 2, y0 + 420 * S, 0, cGray, TA_CENTER);
		{
			const int k = 9;
			float bx0 = x0 + 90 * S, bx1 = x0 + pw - 90 * S, by0 = y0 + 440 * S, by1 = by0 + 27 * S;
			g_menuR[k][0] = bx0; g_menuR[k][1] = by0; g_menuR[k][2] = bx1; g_menuR[k][3] = by1;
			bool sel = (in.menuSel == k);
			hRect(bx0, by0, bx1, by1, sel ? Col{ 0.30f, 0.65f, 1.0f, 0.55f } : Col{ 0.40f, 0.40f, 0.45f, 0.25f });
			swprintf_s(buf, L"%s マウスモード: %s", in.mouseMode ? L"●" : L"○", in.mouseMode ? L"ON（マウスで視点移動）" : L"OFF（I/J/K/Lキーで視点移動）");
			hText(buf, W / 2, by0 + 4 * S, 1, cWhite, TA_CENTER);
		}
		{   // 疑似レイトレーシングのボタン
			const int k = 10;
			const bool pdis = in.shaderFailed || !in.pseudoUnlocked;
			float bx0 = x0 + 90 * S, bx1 = x0 + pw - 90 * S, by0 = y0 + 472 * S, by1 = by0 + 27 * S;
			g_menuR[k][0] = bx0; g_menuR[k][1] = by0; g_menuR[k][2] = bx1; g_menuR[k][3] = by1;
			bool sel = (in.menuSel == k) && !pdis;
			hRect(bx0, by0, bx1, by1, pdis ? Col{ 0.25f, 0.25f, 0.27f, 0.15f } : sel ? Col{ 0.30f, 0.65f, 1.0f, 0.55f } : Col{ 0.40f, 0.40f, 0.45f, 0.25f });
			if (pdis) swprintf_s(buf, in.shaderFailed ? L"× 疑似レイトレーシング（読み込み失敗のため選択不可）" : L"× 疑似レイトレーシング（読み込み中のため選択不可）");
			else swprintf_s(buf, L"%s 疑似レイトレーシング: %s", in.pseudo ? L"●" : L"○", in.pseudo ? L"ON（画面内の光線追跡）" : L"OFF");
			hText(buf, W / 2, by0 + (pdis ? 6 : 4) * S, pdis ? 0 : 1, pdis ? RGB(110, 110, 110) : cWhite, TA_CENTER);
		}
		hText((L"GPU: " + in.gpu).c_str(), W / 2, y0 + ph - 68 * S, 0, cGray, TA_CENTER);
		hText(in.rtNote.c_str(), W / 2, y0 + ph - 50 * S, 0, cGray, TA_CENTER);
		hText(L"↑↓:選択  ←→:時刻を30分ずつ  Enter/クリック:決定(時刻は朝→昼→夕→夜)  Esc:再開", W / 2, y0 + ph - 28 * S, 0, cGray, TA_CENTER);
	}
}


static int  g_handle = -8;
static int  g_cam = 5;
static bool g_quit = false, g_pause = false;
static bool g_driverSeat = false;
static bool g_dead = false;
static bool g_respawnReq = false;
static bool g_interactReq = false;
static bool g_atoReq = false;
static bool g_notchResetReq = false;
static bool g_ato = false;
static bool g_atoSusp = false;
static const int NOTCH_RESET = 0;
static const wchar_t* g_qToast = nullptr; static float g_qToastT = 0.0f;
static void queueToast(const wchar_t* s, float t) { g_qToast = s; g_qToastT = t; }
static bool notchOK() {
	if (g_ato) { queueToast(L"自動運転中はノッチ操作できません（Qで手動へ / Rでリセット）", 1.5f); return false; }
	return true;
}

static bool g_leftDoorReq = false;
static bool g_rightDoorReq = false;

static float g_camYaw = 0.0f;
static float g_camPitch = 0.0f;

static bool g_mouseMode = true;

static int  g_rev = 1;
static double g_tod = 15.0;
static bool g_todFrozen = false;
static float g_nightF = 0.0f;
static V3 g_sunDir = { 0.678f, 0.530f, -0.509f };
static void updateSky(double tod) {
	const double w = 2.0 * PI * (tod - 12.0) / 24.0;
	const V3 Ew = { 0.998f, 0.0f, -0.059f }, Nm = { -0.039f, 0.750f, -0.661f };
	g_sunDir = norm(Ew * (float)sin(w) + Nm * (float)cos(w));
	float t = std::min(1.0f, std::max(0.0f, (g_sunDir.y + 0.18f) / 0.30f));
	g_nightF = 1.0f - t * t * (3.0f - 2.0f * t);
}
static bool lampTime(double tod) {
	const double w = 2.0 * PI * (tod - 12.0) / 24.0;
	const V3 Ew = { 0.998f, 0.0f, -0.059f }, Nm = { -0.039f, 0.750f, -0.661f };
	const V3 sd = norm(Ew * (float)sin(w) + Nm * (float)cos(w));
	const float t = std::min(1.0f, std::max(0.0f, (sd.y + 0.18f) / 0.30f));
	const float nf = 1.0f - t * t * (3.0f - 2.0f * t);
	return (nf - 0.30f) / 0.45f > 0.02f;
}
static double nightSpan(double tod) {
	if (!lampTime(tod)) return 0.0;
	for (int i = 1; i <= 2400; i++) if (!lampTime(fmod(tod + i * 0.01, 24.0))) return i * 0.01;
	return 0.0;
}
static int  g_menuSel = 0;
static float g_gravScale = 1.0f;
static float g_env[4] = { 0, 0, 0, 0 };
static float g_sp0[HL_TRAIN_MAX][4], g_sp1[HL_TRAIN_MAX][4], g_spd[HL_TRAIN_MAX][4], g_spp[4];


// 疑似RT用の処理（段階1）が終わり、疑似RTを選べるか
static bool pseudoOK() { return !g_heavyFailed && g_bgPseudoReady.load(); }
// 「最低」以降の読み込みが終わったか。HW-RT の GPU（段階3が失敗していない）は段階3、それ以外は段階2まで
static bool rtUnlocked() { if (g_heavyFailed) return false; return g_hwRT ? g_bgHwDone.load() : g_bgSwDone.load(); }
static bool gfxOK(int j) {
	if (j == 0) return true;
	if (!rtUnlocked()) return false;
	if (g_hwRT) return j != 4 || !g_swGpu;   // HW-RT の GPU は全モード（WARP の「高」は除く）
	return j <= 3 || (j == 4 && !g_swGpu);   // HW-RT が無い GPU は「最低」〜「高」
}
static void menuApply(int k, int sub = 0) {
	if (k == 0) {
		if (sub != 0) g_tod = fmod(g_tod + sub * 0.5 + 24.0, 24.0);
		else {
			const double ps[4] = { 6.0, 12.0, 18.0, 0.0 };
			double best = 24.0;
			for (double p : ps) { double d = fmod(p - g_tod + 24.0, 24.0); if (d < 1e-6) d = 24.0; best = std::min(best, d); }
			g_tod = fmod(g_tod + best, 24.0);
		}
	}
	else if (k == 1) g_todFrozen = !g_todFrozen;
	else if (k == 9) g_mouseMode = !g_mouseMode;
	else if (k == 10) { if (pseudoOK()) g_pseudoRT = !g_pseudoRT; }
	else { int j = k - 2; if (gfxOK(j)) { g_gfx = j; g_pseudoRT = false; } }
}
static void menuMove(int dir) {
	int k = g_menuSel + dir;
	while (k >= 2 && k <= 8 && !gfxOK(k - 2)) k += dir;
	if (k >= 0 && k <= 10) g_menuSel = k;
}


static bool g_fullscreen = false;
static WINDOWPLACEMENT g_wpPrev = { sizeof(WINDOWPLACEMENT) };
static void toggleFullscreen(HWND hwnd) {
	const LONG_PTR style = GetWindowLongPtrW(hwnd, GWL_STYLE);
	if (!g_fullscreen) {
		MONITORINFO mi = { sizeof(mi) };
		if (GetWindowPlacement(hwnd, &g_wpPrev) && GetMonitorInfoW(MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST), &mi)) {
			SetWindowLongPtrW(hwnd, GWL_STYLE, style & ~(LONG_PTR)WS_OVERLAPPEDWINDOW);
			SetWindowPos(hwnd, HWND_TOP, mi.rcMonitor.left, mi.rcMonitor.top, mi.rcMonitor.right - mi.rcMonitor.left, mi.rcMonitor.bottom - mi.rcMonitor.top,
				SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
			g_fullscreen = true;
		}
	}
	else {
		SetWindowLongPtrW(hwnd, GWL_STYLE, style | WS_OVERLAPPEDWINDOW);
		SetWindowPlacement(hwnd, &g_wpPrev);
		SetWindowPos(hwnd, nullptr, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOOWNERZORDER | SWP_FRAMECHANGED);
		g_fullscreen = false;
	}
}

static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
	switch (msg) {
	case WM_KEYDOWN:
		if (!(lp & (1 << 30))) {
			if (g_pause && (wp == VK_UP || wp == VK_DOWN || wp == VK_RETURN || wp == VK_LEFT || wp == VK_RIGHT)) {
				if (wp == VK_LEFT || wp == VK_RIGHT) { if (g_menuSel == 0) menuApply(0, wp == VK_LEFT ? -1 : 1); }
				else if (wp == VK_UP) menuMove(-1);
				else if (wp == VK_DOWN) menuMove(1);
				else menuApply(g_menuSel);
				return 0;
			}
			switch (wp) {
			case 'W':               if (g_cam != 5 && notchOK()) g_handle = std::min(g_handle + 1, 5); break;
			case 'S':               if (g_cam != 5 && notchOK()) g_handle = std::max(g_handle - 1, -9); break;
			case 'T':               if (g_cam == 5 && notchOK()) g_handle = std::min(g_handle + 1, 5); break;
			case 'G':               if (g_cam == 5 && notchOK()) g_handle = std::max(g_handle - 1, -9); break;
			case VK_UP:             if (notchOK()) g_rev = std::min(g_rev + 1, 1); break;
			case VK_DOWN:           if (notchOK()) g_rev = std::max(g_rev - 1, -1); break;
			case 'B':               g_handle = -9; if (g_ato) g_atoSusp = true; break;
			case 'O': case 'o':     g_leftDoorReq = true; break;
			case 'P': case 'p':     g_rightDoorReq = true; break;
			case '1': if (g_driverSeat) g_cam = 1; break;
			case '2': if (g_driverSeat) g_cam = 5; break;
			case VK_F11: toggleFullscreen(hwnd); break;
			case 'E': g_interactReq = true; break;
			case 'Q': g_atoReq = true; break;
			case 'R': g_notchResetReq = true; break;
			case VK_RETURN: if (g_dead) g_respawnReq = true; break;
			case VK_ESCAPE: g_pause = !g_pause; if (g_pause) g_menuSel = 0; break;
			}
		}
		return 0;
	case WM_MOUSEMOVE: case WM_LBUTTONDOWN:
		if (!g_pause && g_dead) {
			const float mx = (float)(short)LOWORD(lp), my = (float)(short)HIWORD(lp);
			g_respHover = (mx >= g_respR[0] && mx <= g_respR[2] && my >= g_respR[1] && my <= g_respR[3]);
			if (g_respHover && msg == WM_LBUTTONDOWN) g_respawnReq = true;
		}
		if (g_pause) {
			float mx = (float)(short)LOWORD(lp), my = (float)(short)HIWORD(lp);
			for (int k = 0; k < 11; k++)
				if (mx >= g_menuR[k][0] && mx <= g_menuR[k][2] && my >= g_menuR[k][1] && my <= g_menuR[k][3]) {
					if (k >= 2 && k <= 8 && !gfxOK(k - 2)) continue;
					g_menuSel = k;
					if (msg == WM_LBUTTONDOWN) {
						int sub = 0;
						if (k == 0) { const float t = (mx - g_menuR[0][0]) / (g_menuR[0][2] - g_menuR[0][0]); sub = t < 0.25f ? -1 : (t > 0.75f ? 1 : 0); }
						menuApply(k, sub);
					}
				}
		}
		return 0;
	case WM_RBUTTONDOWN:
		g_interactReq = true;
		return 0;
	case WM_SIZE:
		if (g_sc && wp != SIZE_MINIMIZED) onResize();
		return 0;
	case WM_DESTROY: g_quit = true; PostQuitMessage(0); return 0;
	}
	return DefWindowProcW(hwnd, msg, wp, lp);
}


static const UINT HV_MAX = 16384;
static bool g_curTrain = false, g_curGlass = false;
static const V3 SUN_DIR = { 0.678f, 0.530f, -0.509f };

static void drawMesh(const Mesh& m, const M4& vp, const M4& world) {
	(void)vp;
	float c[20]; memcpy(c, &world, sizeof(M4));
	c[16] = g_curTrain ? 1.0f : 0.0f; c[17] = g_curGlass ? 1.0f : 0.0f; c[18] = 0.0f; c[19] = 0.0f;
	g_cl->SetGraphicsRoot32BitConstants(0, 20, c, 0);
	g_cl->IASetVertexBuffers(0, 1, &m.vbv);
	g_cl->DrawInstanced(m.n, 1, m.start, 0);
}
static void setGlassBlend(float f) { const float bf[4] = { f, f, f, f }; g_cl->OMSetBlendFactor(bf); }



struct HouseMeshes { Mesh door[NHD], glass, glow[3], flame, sphere, water, pcStatic, pcBar[6], pcNeedle, pcFill; };
static HouseMeshes g_hm;

static void houseUpdate(float dt) {
	for (int d = 0; d < NHD; d++) {
		const float tg = g_hs.doorTgt[d] ? 1.0f : 0.0f, step = dt / 0.9f;
		g_hs.doorPos[d] += std::min(step, std::max(-step, tg - g_hs.doorPos[d]));
	}
	const float k = std::min(1.0f, dt * 12.0f);
	for (int i = 0; i < 2; i++) g_hs.roomLvl[i] += ((g_hs.roomOn[i] ? 1.0f : 0.0f) - g_hs.roomLvl[i]) * k;
	g_hs.lampLvl += ((g_hs.lampOn ? 1.0f : 0.0f) - g_hs.lampLvl) * k;
	g_hs.tapLvl += ((g_hs.tapOn ? 1.0f : 0.0f) - g_hs.tapLvl) * std::min(1.0f, dt * 6.0f);
	g_hs.pcLvl += ((g_hs.pcOn ? 1.0f : 0.0f) - g_hs.pcLvl) * k;
	g_hs.stoveLvl += ((g_hs.stoveOn ? 1.0f : 0.0f) - g_hs.stoveLvl) * k;
	g_hs.steam += ((g_hs.stoveOn ? 1.0f : 0.0f) - g_hs.steam) * std::min(1.0f, dt * (g_hs.stoveOn ? 0.8f : 0.5f));
}
static void houseFillCB(FrameCB& cb, float tSec) {
	if (!g_hs.valid) return;
	cb.hlO[0] = g_hs.O.x; cb.hlO[1] = g_hs.O.y; cb.hlO[2] = g_hs.O.z; cb.hlO[3] = 1.0f;
	cb.hlRot[0] = g_hs.R.x; cb.hlRot[1] = g_hs.R.z; cb.hlRot[2] = g_hs.F.x; cb.hlRot[3] = g_hs.F.z;
	const float lv[4] = { g_hs.roomLvl[0], g_hs.roomLvl[1], g_hs.lampLvl, g_hs.stoveLvl };
	for (int k = 0; k < 4; k++) {
		const HLightDef& L = H_LIGHTS[k];
		const V3 p = hzW(L.pos);
		const float fl = (k == 3) ? 0.85f + 0.15f * sinf(tSec * 17.0f) * sinf(tSec * 5.3f + 1.0f) : 1.0f;
		cb.hlP[k][0] = p.x; cb.hlP[k][1] = p.y; cb.hlP[k][2] = p.z; cb.hlP[k][3] = L.range;
		cb.hlC[k][0] = L.col.x * lv[k] * fl; cb.hlC[k][1] = L.col.y * lv[k] * fl; cb.hlC[k][2] = L.col.z * lv[k] * fl; cb.hlC[k][3] = L.down;
		cb.hlMn[k][0] = L.mn.x; cb.hlMn[k][1] = L.mn.y; cb.hlMn[k][2] = L.mn.z; cb.hlMn[k][3] = (k < 2) ? lv[k] : 0.0f;
		cb.hlMx[k][0] = L.mx.x; cb.hlMx[k][1] = L.mx.y; cb.hlMx[k][2] = L.mx.z; cb.hlMx[k][3] = (k == 0) ? 1.0f : ((k == 1) ? 0.8f : 0.0f);
	}
}

static void houseDrawOpaque(const M4& vp, float tSec) {
	if (!g_hs.valid) return;
	for (int d = 0; d < NHD; d++) {
		const HDoorDef& D = H_DOORS[d];
		const float p = g_hs.doorPos[d], ang = p * p * (3.0f - 2.0f * p) * D.maxAng;
		const V3 Rd = D.closedDir * cosf(ang) + D.openDir * sinf(ang), Fd = cross(Rd, { 0, 1, 0 });
		drawMesh(g_hm.door[d], vp, mul(basisMat(Rd, { 0, 1, 0 }, Fd, D.hinge), g_hs.M));
	}
	if (g_hs.roomLvl[0] > 0.3f) drawMesh(g_hm.glow[0], vp, g_hs.M);
	if (g_hs.roomLvl[1] > 0.3f) drawMesh(g_hm.glow[1], vp, g_hs.M);
	if (g_hs.lampLvl > 0.3f) drawMesh(g_hm.glow[2], vp, g_hs.M);
	if (g_hs.pcLvl > 0.3f) {
		const float PW = PC_X1 - PC_X0, PH = PC_Y1 - PC_Y0;
		drawMesh(g_hm.pcStatic, vp, g_hs.M);
		for (int k = 0; k < 6; k++) {
			const float bh = PH * (0.06f + 0.44f * (0.5f + 0.5f * sinf(tSec * 1.9f + k * 1.15f)));
			drawMesh(g_hm.pcBar[k], vp, mul(basisMat({ PW * 0.05f, 0, 0 }, { 0, bh, 0 }, { 0, 0, 1 }, { PC_X0 + (0.14f + 0.08f * k) * PW, PC_Y0 + 0.262f * PH, 0 }), g_hs.M));
		}
		const float na = tSec * 2.2f, ca = cosf(na), sa = sinf(na);
		drawMesh(g_hm.pcNeedle, vp, mul(basisMat({ ca, sa, 0 }, { -sa, ca, 0 }, { 0, 0, 1 }, { PC_X0 + 0.80f * PW, PC_Y0 + 0.58f * PH, 0 }), g_hs.M));
		const float fw = PW * 0.24f * fmodf(tSec * 0.25f, 1.0f);
		drawMesh(g_hm.pcFill, vp, mul(basisMat({ fw, 0, 0 }, { 0, PH * 0.05f, 0 }, { 0, 0, 1 }, { PC_X0 + 0.68f * PW, PC_Y0 + 0.30f * PH, 0 }), g_hs.M));
	}
	if (g_hs.stoveLvl > 0.15f) {
		const float fl = 0.85f + 0.15f * sinf(tSec * 23.0f) + 0.10f * sinf(tSec * 37.0f + 1.3f), sy = fl * std::min(1.0f, g_hs.stoveLvl);
		drawMesh(g_hm.flame, vp, mul(basisMat({ 1, 0, 0 }, { 0, sy, 0 }, { 0, 0, 1 }, { -1.85f, 0.93f, -3.5f }), g_hs.M));
	}
}

static void houseDrawGlass(const M4& vp, float tSec, const V3& camRight) {
	(void)camRight;
	if (!g_hs.valid) return;
	setGlassBlend(0.30f);
	drawMesh(g_hm.glass, vp, g_hs.M);
	if (g_hs.tapLvl > 0.02f) {
		const float lv = std::min(1.0f, g_hs.tapLvl);
		for (int k = 0; k < 3; k++) {
			const float ph = tSec * 11.0f + k * 2.1f;
			setGlassBlend(0.26f * lv);
			drawMesh(g_hm.water, vp, mul(basisMat({ 1, 0, 0 }, { 0, lv, 0 }, { 0, 0, 1 }, { -2.55f + 0.004f * sinf(ph), 1.06f - 0.125f * lv, -3.565f + 0.004f * cosf(ph * 1.3f) }), g_hs.M));
		}
		const float sr = 0.030f + 0.008f * sinf(tSec * 17.0f);
		setGlassBlend(0.34f * lv);
		drawMesh(g_hm.sphere, vp, mul(basisMat({ sr, 0, 0 }, { 0, 0.012f, 0 }, { 0, 0, sr }, { -2.55f, 0.943f, -3.565f }), g_hs.M));
	}
	if (g_hs.steam > 0.02f) {
		const int N = 14;
		for (int i = 0; i < N; i++) {
			const float ph = fmodf(tSec * 0.32f + (float)i / N, 1.0f), a = sinf(PI * ph);
			const float r = 0.045f + 0.12f * ph, sw = 0.05f * (1.0f + ph);
			const V3 pos = { -1.85f + sw * sinf(ph * 6.0f + i * 1.9f), 1.10f + ph * 0.80f, -3.5f + sw * cosf(ph * 5.0f + i * 2.3f) };
			setGlassBlend(0.20f * a * g_hs.steam);
			drawMesh(g_hm.sphere, vp, mul(basisMat({ r, 0, 0 }, { 0, r, 0 }, { 0, 0, r }, pos), g_hs.M));
		}
		setGlassBlend(0.30f);
	}
}


static void hudRasterText() {
	memset(g_bits, 0, (size_t)g_w * g_h * 4);
	SetBkMode(g_hdc, TRANSPARENT);
	for (const TItem& t : g_tx) {
		SelectObject(g_hdc, g_font[t.font]);
		SetTextColor(g_hdc, t.col);
		SetTextAlign(g_hdc, t.align | TA_TOP);
		TextOutW(g_hdc, t.x, t.y, t.s.c_str(), (int)t.s.size());
	}
	GdiFlush();
}


static void appendLeaves(Verts& v) {
	Verts lf[2] = { buildDoorLeaf(-1.0f), buildDoorLeaf(1.0f) };
	for (int s = 0; s < 2; s++) for (int k = 0; k < 3; k++) for (int dir = -1; dir <= 1; dir += 2) {
		float oz = DOOR_Z[k] + dir * LEAF_OFS;
		for (Vtx x : lf[s]) { x.p.z += oz; v.push_back(x); }
	}
}


static void appendCabDoors(Verts& v, float zs) {
	for (int s = -1; s <= 1; s += 2) { Verts d = buildCabDoor((float)s, zs); v.insert(v.end(), d.begin(), d.end()); }
}






static const float FP_R = 0.25f;
static const float FP_STEP = 0.29f;
static const float FP_EYE = 1.62f;
static const float FP_EYE_SIT = 0.90f;
static const float HIT_MIN_KMH = 5.0f;
static const float FP_WALK = 5.0f;
static const float FP_DASH = 11.1f;
static const float FP_MOUSE_SENS = 0.0022f;
static const float FP_HS[3] = { 0.55f, 1.05f, 1.55f };

static V3 xfPoint(const M4& m, V3 p) {
	return { p.x * m.m[0][0] + p.y * m.m[1][0] + p.z * m.m[2][0] + m.m[3][0],
			 p.x * m.m[0][1] + p.y * m.m[1][1] + p.z * m.m[2][1] + m.m[3][1],
			 p.x * m.m[0][2] + p.y * m.m[1][2] + p.z * m.m[2][2] + m.m[3][2] };
}

static V3 closestPtTri(V3 p, V3 a, V3 b, V3 c) {
	V3 ab = b - a, ac = c - a, ap = p - a;
	float d1 = dot(ab, ap), d2 = dot(ac, ap);
	if (d1 <= 0 && d2 <= 0) return a;
	V3 bp = p - b; float d3 = dot(ab, bp), d4 = dot(ac, bp);
	if (d3 >= 0 && d4 <= d3) return b;
	float vc = d1 * d4 - d3 * d2;
	if (vc <= 0 && d1 >= 0 && d3 <= 0) return a + ab * (d1 / (d1 - d3));
	V3 cp = p - c; float d5 = dot(ab, cp), d6 = dot(ac, cp);
	if (d6 >= 0 && d5 <= d6) return c;
	float vb = d5 * d2 - d1 * d6;
	if (vb <= 0 && d2 >= 0 && d6 <= 0) return a + ac * (d2 / (d2 - d6));
	float va = d3 * d6 - d5 * d4;
	if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
	float den = 1.0f / (va + vb + vc);
	return a + ab * (vb * den) + ac * (vc * den);
}

struct CTri { V3 a, b, c; uint8_t kind; };
struct ColGrid {
	std::vector<CTri> tris;
	std::vector<uint32_t> cellStart, items, big;
	float ox = 0, oz = 0, cs = 1, inv = 1; int nx = 0, nz = 0;

	void build(const Verts& v, float cellSize, bool skipWater) {
		cs = cellSize; inv = 1.0f / cs;
		tris.clear(); cellStart.clear(); items.clear(); big.clear();
		tris.reserve(v.size() / 3);
		for (size_t i = 0; i + 2 < v.size(); i += 3) {
			const Vtx& p0 = v[i]; const Vtx& p1 = v[i + 1]; const Vtx& p2 = v[i + 2];
			if (skipWater && fabsf(p0.p.y - WATER_Y) < 1e-3f && fabsf(p1.p.y - WATER_Y) < 1e-3f && fabsf(p2.p.y - WATER_Y) < 1e-3f && p0.c.z > 0.5f && p0.c.x < 0.13f)
				continue;
			V3 n = cross(p1.p - p0.p, p2.p - p0.p);
			float l2 = dot(n, n);
			if (l2 < 1e-10f) continue;
			n = n * (1.0f / sqrtf(l2));
			uint8_t k = 0;
			if (fabsf(n.y) >= 0.55f) {
				if ((p0.n.y + p1.n.y + p2.n.y) < 0.0f) continue;
				k = 1;
			}
			tris.push_back({ p0.p, p1.p, p2.p, k });
		}
		const size_t N = tris.size();
		float gx0 = 1e30f, gz0 = 1e30f, gx1 = -1e30f, gz1 = -1e30f;
		std::vector<uint8_t> isBig(N, 0);
		for (size_t i = 0; i < N; i++) {
			const CTri& t = tris[i];
			float x0 = std::min(t.a.x, std::min(t.b.x, t.c.x)), x1 = std::max(t.a.x, std::max(t.b.x, t.c.x));
			float z0 = std::min(t.a.z, std::min(t.b.z, t.c.z)), z1 = std::max(t.a.z, std::max(t.b.z, t.c.z));
			if (((x1 - x0) * inv + 1.0f) * ((z1 - z0) * inv + 1.0f) > 400.0f) { isBig[i] = 1; big.push_back((uint32_t)i); continue; }
			gx0 = std::min(gx0, x0); gx1 = std::max(gx1, x1); gz0 = std::min(gz0, z0); gz1 = std::max(gz1, z1);
		}
		if (gx0 > gx1) { nx = nz = 0; cellStart.assign(1, 0); return; }
		ox = gx0; oz = gz0; nx = (int)((gx1 - gx0) * inv) + 1; nz = (int)((gz1 - gz0) * inv) + 1;
		cellStart.assign((size_t)nx * nz + 1, 0);
		auto range = [&](const CTri& t, int& ix0, int& ix1, int& iz0, int& iz1) {
			float x0 = std::min(t.a.x, std::min(t.b.x, t.c.x)), x1 = std::max(t.a.x, std::max(t.b.x, t.c.x));
			float z0 = std::min(t.a.z, std::min(t.b.z, t.c.z)), z1 = std::max(t.a.z, std::max(t.b.z, t.c.z));
			ix0 = std::max(0, (int)floorf((x0 - ox) * inv)); ix1 = std::min(nx - 1, (int)floorf((x1 - ox) * inv));
			iz0 = std::max(0, (int)floorf((z0 - oz) * inv)); iz1 = std::min(nz - 1, (int)floorf((z1 - oz) * inv));
			};
		for (size_t i = 0; i < N; i++) {
			if (isBig[i]) continue;
			int a, b, c, d; range(tris[i], a, b, c, d);
			for (int iz = c; iz <= d; iz++) for (int ix = a; ix <= b; ix++) cellStart[(size_t)iz * nx + ix + 1]++;
		}
		for (size_t i = 1; i < cellStart.size(); i++) cellStart[i] += cellStart[i - 1];
		items.resize(cellStart.back());
		std::vector<uint32_t> cur(cellStart.begin(), cellStart.end() - 1);
		for (size_t i = 0; i < N; i++) {
			if (isBig[i]) continue;
			int a, b, c, d; range(tris[i], a, b, c, d);
			for (int iz = c; iz <= d; iz++) for (int ix = a; ix <= b; ix++) items[cur[(size_t)iz * nx + ix]++] = (uint32_t)i;
		}
	}

	template<class F> void each(float x0, float z0, float x1, float z1, F&& f) const {
		for (uint32_t i : big) f(tris[i]);
		if (nx == 0) return;
		int ix0 = std::max(0, (int)floorf((x0 - ox) * inv)), ix1 = std::min(nx - 1, (int)floorf((x1 - ox) * inv));
		int iz0 = std::max(0, (int)floorf((z0 - oz) * inv)), iz1 = std::min(nz - 1, (int)floorf((z1 - oz) * inv));
		for (int iz = iz0; iz <= iz1; iz++) for (int ix = ix0; ix <= ix1; ix++) {
			const size_t cell = (size_t)iz * nx + ix;
			for (uint32_t k = cellStart[cell]; k < cellStart[cell + 1]; k++) f(tris[items[k]]);
		}
	}


	bool pushOut(V3& c, float r, float feetY) const {
		bool moved = false;
		const float r2 = r * r, thr = feetY + FP_STEP + 0.01f;
		each(c.x - r, c.z - r, c.x + r, c.z + r, [&](const CTri& t) {
			float mn = std::min(t.a.y, std::min(t.b.y, t.c.y)), mx = std::max(t.a.y, std::max(t.b.y, t.c.y));
			if (mn > c.y + r || mx < c.y - r) return;
			V3 q = closestPtTri(c, t.a, t.b, t.c);
			if (t.kind == 1 && q.y <= thr) return;
			V3 d = c - q; float d2 = dot(d, d);
			if (d2 >= r2) return;
			V3 h;
			if (d2 > 1e-10f) { float dl = sqrtf(d2); h = d * ((r - dl) / dl); }
			else { V3 n = norm(cross(t.b - t.a, t.c - t.a)); if (dot(c - t.a, n) < 0) n = n * -1.0f; h = n * r; }
			h.y = 0.0f;
			c = c + h; moved = true;
			});
		return moved;
	}


	float groundAt(float x, float z, float yMax) const {
		float best = -1e30f;
		each(x, z, x, z, [&](const CTri& t) {
			if (t.kind != 1) return;
			if (std::min(t.a.y, std::min(t.b.y, t.c.y)) > yMax) return;
			float d = (t.b.z - t.c.z) * (t.a.x - t.c.x) + (t.c.x - t.b.x) * (t.a.z - t.c.z);
			if (fabsf(d) < 1e-9f) return;
			float l1 = ((t.b.z - t.c.z) * (x - t.c.x) + (t.c.x - t.b.x) * (z - t.c.z)) / d;
			float l2 = ((t.c.z - t.a.z) * (x - t.c.x) + (t.a.x - t.c.x) * (z - t.c.z)) / d;
			float l3 = 1.0f - l1 - l2;
			const float e = -1e-4f;
			if (l1 < e || l2 < e || l3 < e) return;
			float y = l1 * t.a.y + l2 * t.b.y + l3 * t.c.y;
			if (y <= yMax && y > best) best = y;
			});
		return best;
	}


	float rayFirst(V3 o, V3 d, float maxT) const {
		float best = 1e30f;
		const V3 e = o + d * maxT;
		each(std::min(o.x, e.x) - 0.05f, std::min(o.z, e.z) - 0.05f, std::max(o.x, e.x) + 0.05f, std::max(o.z, e.z) + 0.05f, [&](const CTri& t) {
			const V3 e1 = t.b - t.a, e2 = t.c - t.a, pv = cross(d, e2);
			const float det = dot(e1, pv);
			if (fabsf(det) < 1e-9f) return;
			const float idet = 1.0f / det;
			const V3 tv = o - t.a;
			const float u = dot(tv, pv) * idet; if (u < 0.0f || u > 1.0f) return;
			const V3 qv = cross(tv, e1);
			const float w = dot(d, qv) * idet; if (w < 0.0f || u + w > 1.0f) return;
			const float tt = dot(e2, qv) * idet;
			if (tt > 1e-4f && tt < best && tt <= maxT) best = tt;
			});
		return best;
	}
};

static ColGrid g_colWorld;
static ColGrid g_colCar[3];
static ColGrid g_colLeaf[2];
static ColGrid g_colHDoor[NHD];
static ColGrid g_colCab[2][2];

static void buildCollision(const Verts& worldAll) {
	Verts world; world.reserve(worldAll.size());
	for (size_t i = 0; i + 2 < worldAll.size(); i += 3) {
		bool skip = false;
		for (const auto& bx : g_noColBoxes) {
			bool all = true;
			for (int k = 0; k < 3; k++) { const V3& q = worldAll[i + k].p; if (q.x < bx.first.x || q.y < bx.first.y || q.z < bx.first.z || q.x > bx.second.x || q.y > bx.second.y || q.z > bx.second.z) { all = false; break; } }
			if (all) { skip = true; break; }
		}
		if (!skip) { world.push_back(worldAll[i]); world.push_back(worldAll[i + 1]); world.push_back(worldAll[i + 2]); }
	}
	g_colWorld.build(world, 8.0f, true);
	if (g_hs.valid) for (int d = 0; d < NHD; d++) g_colHDoor[d].build(hzDoorWorld(d, 0.0f), 0.5f, false);
	const bool hd[3] = { true, false, false }, tl[3] = { false, false, true };
	for (int k = 0; k < 3; k++) {
		Verts v = buildCarBody(hd[k], tl[k]);
		Verts g = buildCarGlass(hd[k], tl[k]); v.insert(v.end(), g.begin(), g.end());
		if (hd[k]) { Verts w = buildWindscreen(1.0f); v.insert(v.end(), w.begin(), w.end()); }
		if (tl[k]) { Verts w = buildWindscreen(-1.0f); v.insert(v.end(), w.begin(), w.end()); }
		g_colCar[k].build(v, 0.5f, false);
	}
	for (int s = 0; s < 2; s++) {
		const float sg = s ? 1.0f : -1.0f;
		Verts lf = buildDoorLeaf(sg), gl = buildDoorGlass(sg), v;
		for (int k = 0; k < 3; k++) for (int dir = -1; dir <= 1; dir += 2) {
			const float oz = DOOR_Z[k] + dir * LEAF_OFS;
			for (Vtx x : lf) { x.p.z += oz; v.push_back(x); }
			for (Vtx x : gl) { x.p.z += oz; v.push_back(x); }
		}
		g_colLeaf[s].build(v, 0.5f, false);
	}
	for (int s = 0; s < 2; s++) for (int e = 0; e < 2; e++) {
		const float sg = s ? 1.0f : -1.0f, zs = e ? -1.0f : 1.0f;
		Verts v = buildCabDoor(sg, zs), g = buildCabDoorGlass(sg, zs);
		v.insert(v.end(), g.begin(), g.end());
		g_colCab[s][e].build(v, 0.5f, false);
	}
}

struct FPState {
	bool valid = false;
	V3 pos = { 0, 0, 0 };
	float vy = 0.0f;
	bool grounded = false;
	float yaw = 0.0f, pitch = 0.0f;
	int car = -1;
	V3 local = { 0, 0, 0 };
	float heading = 0.0f;
	bool resume = false;
	float eyeY = 0.0f;
	int seat = 0;
	int seatCar = -1;
	V3 seatTgt = { 0, 0, 0 };
	V3 standLocal = { 0, 0, 0 };
	float seatYaw = 0.0f;
	V3 airVel = { 0, 0, 0 };
	float rideGrace = 0.0f;
};
static FPState g_fp;
static float g_trainV = 0.0f;
static const float FP_JUMP = 5.5f;
static const float FP_EYE_LIE = 0.30f;
static float g_fpHitKmh = -1.0f;


static void buildBenchList() {}


struct FPAim {
	bool valid = false;
	int kind = 0;
	int car = -1;
	V3 sit = { 0, 0, 0 }, stand = { 0, 0, 0 };
	float yaw = 0.0f;
	int idx = 0;
	const wchar_t* prompt = L"";
};
static FPAim g_aim;

static float g_cabPos[4] = { 0, 0, 0, 0 };
static bool  g_cabTgt[4] = { false, false, false, false };
static FPAim g_act;


static void fpSpawn(float unused, int forceStation = -1) {
	(void)unused; (void)forceStation;
	const StationData& st = g_stations[0];
	PfDef pd[2]; stationPlatforms(0, pd);
	TFrame F = trackFrame(st.platformS0 + 52.0f);
	V3 p = F.p + F.r * ((pd[0].a + pd[0].b) * 0.5f) + F.u * (PF_TOP + 0.1f);
	g_fp.pos = p; g_fp.vy = 0.0f; g_fp.grounded = false;
	g_fp.yaw = atan2f(F.f.x, F.f.z); g_fp.pitch = 0.0f;
	if (g_hs.valid) {
		p = hzW(H_BED_STAND); p.y += 0.1f;
		const V3 fd = hzWDir({ -1.0f, 0.0f, 0.0f });
		g_fp.pos = p; g_fp.yaw = atan2f(fd.x, fd.z);
	}
	g_fp.car = -1; g_fp.eyeY = g_fp.pos.y; g_fp.valid = true; g_fp.resume = false;
	g_fp.seat = 0; g_fp.seatCar = -1; g_aim = FPAim();
}

static bool fpNearCar(const M4& cw, V3 p) {
	return fabsf(p.x - cw.m[3][0]) < 14.0f && fabsf(p.z - cw.m[3][2]) < 14.0f && fabsf(p.y - cw.m[3][1]) < 9.0f;
}
static int fpCarGrids(int i, float dl, float dr, const float* cab, const ColGrid** out) {
	int n = 0;
	out[n++] = &g_colCar[i == 0 ? 0 : (i == NUM_CARS - 1 ? 2 : 1)];
	if (dl < 0.6f) out[n++] = &g_colLeaf[0];
	if (dr < 0.6f) out[n++] = &g_colLeaf[1];
	if (i == 0 || i == NUM_CARS - 1) {
		const int en = (i == 0) ? 0 : 1;
		for (int s = 0; s < 2; s++) if (cab[en * 2 + s] < 0.4f) out[n++] = &g_colCab[s][en];
	}
	return n;
}


static void fpCollide(const M4* cw, const M4* inv, float dl, float dr, const float* cab) {
	FPState& f = g_fp;
	for (int it = 0; it < 2; it++) for (int s = 0; s < 3; s++) {
		V3 c = { f.pos.x, f.pos.y + FP_HS[s], f.pos.z };
		g_colWorld.pushOut(c, FP_R, f.pos.y);
		f.pos.x = c.x; f.pos.z = c.z;
		if (g_hs.valid) for (int d = 0; d < NHD; d++) if (d >= 2 || g_hs.doorPos[d] < 0.6f) {
			c = { f.pos.x, f.pos.y + FP_HS[s], f.pos.z };
			g_colHDoor[d].pushOut(c, FP_R, f.pos.y);
			f.pos.x = c.x; f.pos.z = c.z;
		}
		for (int i = 0; i < NUM_CARS; i++) {
			if (!fpNearCar(cw[i], f.pos)) continue;
			c = { f.pos.x, f.pos.y + FP_HS[s], f.pos.z };
			V3 lc = xfPoint(inv[i], c), l0 = lc;
			const float lfeet = lc.y - FP_HS[s];
			const ColGrid* gs[8]; const int n = fpCarGrids(i, dl, dr, cab, gs);
			for (int k = 0; k < n; k++) gs[k]->pushOut(lc, FP_R, lfeet);
			V3 d = lc - l0;
			if (d.x != 0.0f || d.z != 0.0f) {
				f.pos.x += d.x * cw[i].m[0][0] + d.z * cw[i].m[2][0];
				f.pos.z += d.x * cw[i].m[0][2] + d.z * cw[i].m[2][2];
			}
		}
	}
}


static float fpGroundAt(float x, float z, float yMax, const M4* cw, const M4* inv, int& src) {
	float gw = g_colWorld.groundAt(x, z, yMax);
	float gc = -1e30f; int ci = -1;
	for (int i = 0; i < NUM_CARS; i++) {
		if (!fpNearCar(cw[i], { x, yMax, z })) continue;
		V3 P = xfPoint(inv[i], { x, yMax, z });
		float g = g_colCar[i == 0 ? 0 : (i == NUM_CARS - 1 ? 2 : 1)].groundAt(P.x, P.z, P.y);
		if (g < -1e29f) continue;
		float wy = xfPoint(cw[i], { P.x, g, P.z }).y;
		if (wy > gc) { gc = wy; ci = i; }
	}
	if (ci >= 0 && gc >= gw - 0.02f) { src = ci; return gc; }
	src = -1; return gw;
}

static void fpGround(float dts, const M4* cw, const M4* inv, int& srcCenter) {
	FPState& f = g_fp;
	const float yMax = f.pos.y + FP_STEP, rs = 0.2f;
	const float ox[5] = { 0, rs, -rs, 0, 0 }, oz[5] = { 0, 0, 0, rs, -rs };
	float best = -1e30f, ringBest = -1e30f; int ringSrc = -1;
	for (int k = 0; k < 5; k++) {
		int s; float g = fpGroundAt(f.pos.x + ox[k], f.pos.z + oz[k], yMax, cw, inv, s);
		if (k == 0) srcCenter = s;
		else if (s >= 0 && g > ringBest) { ringBest = g; ringSrc = s; }
		best = std::max(best, g);
	}

	if (srcCenter < 0 && f.car >= 0 && ringSrc >= 0 && ringBest >= best - 0.02f) srcCenter = ringSrc;
	if (f.grounded && f.vy <= 0.0f && f.pos.y - best <= 0.35f) { f.pos.y = best; f.vy = 0.0f; }
	else {
		f.vy -= 20.0f * dts;
		float ny = f.pos.y + f.vy * dts;
		if (ny <= best) { f.pos.y = best; f.vy = 0.0f; f.grounded = true; }
		else { f.pos.y = ny; f.grounded = false; }
	}
}



static int hitDamage(float kmh) {
	if (kmh >= 90.0f) return 1000;
	if (kmh >= 60.0f) return 400 + (int)floorf((kmh - 60.0f) / 30.0f * 20.0f) * 10;
	if (kmh >= HIT_MIN_KMH) return (int)floorf(kmh * 0.5f + 0.5f) * 10;
	return 0;
}


static bool rayBox(V3 o, V3 d, V3 mn, V3 mx, float& tOut) {
	float t0 = 0.0f, t1 = 1e30f;
	const float oo[3] = { o.x, o.y, o.z }, dd[3] = { d.x, d.y, d.z }, a[3] = { mn.x, mn.y, mn.z }, b[3] = { mx.x, mx.y, mx.z };
	for (int k = 0; k < 3; k++) {
		if (fabsf(dd[k]) < 1e-8f) { if (oo[k] < a[k] || oo[k] > b[k]) return false; continue; }
		float ta = (a[k] - oo[k]) / dd[k], tb = (b[k] - oo[k]) / dd[k];
		if (ta > tb) std::swap(ta, tb);
		t0 = std::max(t0, ta); t1 = std::min(t1, tb);
		if (t0 > t1) return false;
	}
	tOut = t0;
	return true;
}


static void houseAim(V3 eye, V3 dir, float& best) {
	const V3 o = hzL(eye), d = norm(hzLDir(dir));
	if (fabsf(o.x) > 8.5f || o.z < -8.5f || o.z > 9.0f || o.y < -3.0f || o.y > 9.0f) return;
	auto hit = [&](V3 mn, V3 mx) -> bool {
		float t;
		if (!rayBox(o, d, mn, mx, t) || t >= best) return false;
		if (t > 0.03f && g_colWorld.rayFirst(eye, dir, t - 0.03f) < 1e29f) return false;
		best = t; return true;
		};
	for (const HTarget& T : H_TGT) if (hit(T.mn, T.mx)) {
		g_aim = FPAim(); g_aim.valid = true; g_aim.kind = T.kind; g_aim.idx = T.idx;
		if (T.kind == 11) g_aim.prompt = g_hs.roomOn[T.idx] ? L"[E/右クリック] 電気を消す" : L"[E/右クリック] 電気をつける";
		else if (T.kind == 12) g_aim.prompt = g_hs.stoveOn ? L"[E/右クリック] 火を消す" : L"[E/右クリック] コンロに火をつける";
		else if (T.kind == 13) g_aim.prompt = g_hs.lampOn ? L"[E/右クリック] デスクランプを消す" : L"[E/右クリック] デスクランプをつける";
		else if (T.kind == 14) g_aim.prompt = g_hs.tapOn ? L"[E/右クリック] 水を止める" : L"[E/右クリック] 水を出す";
		else g_aim.prompt = g_hs.pcOn ? L"[E/右クリック] パソコンの画面を消す" : L"[E/右クリック] パソコンの画面をつける";
	}
	for (int k = 0; k < H_NCHAIR; k++) {
		const HChair& C = H_CHAIRS[k];
		if (!hit({ C.x - 0.26f, C.yb, C.z - 0.26f }, { C.x + 0.26f, C.yb + 1.0f, C.z + 0.26f })) continue;
		const V3 fd = hzWDir({ 0, 0, C.s });
		g_aim = FPAim(); g_aim.valid = true; g_aim.kind = 5; g_aim.idx = k; g_aim.car = -1;
		g_aim.sit = hzW({ C.x, C.yb + 0.47f, C.z + C.s * 0.02f }); g_aim.stand = hzW({ C.sx, C.yb + 0.02f, C.sz });
		g_aim.yaw = atan2f(fd.x, fd.z); g_aim.prompt = L"[E/右クリック] 椅子に座る";
	}
	if (hit({ -4.30f, 2.75f, -3.80f }, { -3.05f, 3.95f, -1.60f })) {
		const V3 fd = hzWDir({ 0, 0, 1 });
		g_aim = FPAim(); g_aim.valid = true; g_aim.kind = 6; g_aim.car = -1;
		g_aim.sit = hzW(H_BED_SIT); g_aim.stand = hzW(H_BED_STAND);
		g_aim.yaw = atan2f(fd.x, fd.z); g_aim.prompt = L"[E/右クリック] ベッドで寝る";
	}
	for (int k = 0; k < NHD; k++) {
		const bool open = g_hs.doorTgt[k] || g_hs.doorPos[k] > 0.05f;
		if (k >= 2) {
			const float y0 = (k == 2) ? 0.0f : 1.21f, y1 = (k == 2) ? 1.21f : 1.82f;
			bool hh;
			if (!open) hh = hit({ -4.30f, y0, -3.20f }, { -3.55f, y1, -3.00f });
			else { const bool a1 = hit({ -4.34f, y0, -3.20f }, { -4.14f, y1, -2.42f }); const bool a2 = hit({ -4.22f, y0, -3.78f }, { -3.62f, y1, -3.14f }); hh = a1 || a2; }
			if (hh) {
				g_aim = FPAim(); g_aim.valid = true; g_aim.kind = 10; g_aim.idx = k;
				g_aim.prompt = g_hs.doorTgt[k] ? ((k == 2) ? L"[E/右クリック] 冷蔵室を閉める" : L"[E/右クリック] 冷凍室を閉める") : ((k == 2) ? L"[E/右クリック] 冷蔵室を開ける" : L"[E/右クリック] 冷凍室を開ける");
			}
			continue;
		}
		V3 mn, mx;
		if (k == 0) { mn = { 2.60f, 0.0f, open ? 2.80f : 3.60f }; mx = { 3.80f, 2.20f, 4.20f }; }
		else { mn = { open ? 2.05f : 2.90f, 2.75f, -3.65f }; mx = { 3.25f, 4.90f, -2.60f }; }
		if (hit(mn, mx)) {
			g_aim = FPAim(); g_aim.valid = true; g_aim.kind = 10; g_aim.idx = k;
			g_aim.prompt = g_hs.doorTgt[k] ? L"[E/右クリック] ドアを閉める" : L"[E/右クリック] ドアを開ける";
		}
	}
}



static void fpAim(const M4* cw, const M4* inv, float dl, float dr, const float* cab, V3 eye, V3 dir) {
	g_aim = FPAim();
	float best = 3.0f;
	for (int i = 0; i < NUM_CARS; i++) {
		if (!fpNearCar(cw[i], eye)) continue;
		const V3 o = xfPoint(inv[i], eye);
		if (fabsf(o.x) > 1.45f || fabsf(o.z) > 10.1f || o.y < Y_FLOOR || o.y > 3.4f) continue;
		const V3 d = norm(xfPoint(inv[i], eye + dir) - o);
		const bool head = (i == 0), tail = (i == NUM_CARS - 1);
		const int gi = head ? 0 : (tail ? 2 : 1);
		auto blocked = [&](float t) -> bool {
			const float lim = t - 0.03f;
			if (lim <= 0.0f) return false;
			if (g_colCar[gi].rayFirst(o, d, lim) < 1e29f) return true;
			if (dl < 0.6f && g_colLeaf[0].rayFirst(o, d, lim) < 1e29f) return true;
			if (dr < 0.6f && g_colLeaf[1].rayFirst(o, d, lim) < 1e29f) return true;
			if (head || tail) for (int s = 0; s < 2; s++)
				if (cab[(head ? 0 : 1) * 2 + s] < 0.4f && g_colCab[s][head ? 0 : 1].rayFirst(o, d, lim) < 1e29f) return true;
			return false;
			};


		const bool inCab = head ? (o.z > 8.0f) : (tail ? (o.z < -8.0f) : false);
		if (!inCab) {
			const float segs[4][2] = { { -9.9f, -DOOR_Z[2] - DOOR_HW }, { -5.35f, -0.65f }, { 0.65f, 5.35f }, { DOOR_Z[2] + DOOR_HW, 9.9f } };
			for (int sgi = -1; sgi <= 1; sgi += 2) {
				const float sg = (float)sgi;
				for (int k = 0; k < 4; k++) {
					float a2 = segs[k][0] + 0.15f, b2 = segs[k][1] - 0.15f;
					if (head && k == 3) b2 = 7.9f;
					if (tail && k == 0) a2 = -7.9f;
					float t0 = 1e30f, t1 = 1e30f, tc;
					if (rayBox(o, d, { std::min(sg * 0.80f, sg * 1.19f), Y_FLOOR, a2 }, { std::max(sg * 0.80f, sg * 1.19f), 1.42f, b2 }, tc)) t0 = tc;
					if (rayBox(o, d, { std::min(sg * 1.19f, sg * 1.25f), 1.42f, a2 }, { std::max(sg * 1.19f, sg * 1.25f), 2.00f, b2 }, tc)) t1 = tc;
					const float t = std::min(t0, t1);
					if (t >= best || blocked(t)) continue;
					best = t;
					const V3 hp = o + d * t;
					const float zlo = a2 + 0.30f, zhi = b2 - 0.30f;
					const float zc = (zlo < zhi) ? std::min(zhi, std::max(zlo, hp.z)) : (a2 + b2) * 0.5f;
					g_aim.valid = true; g_aim.kind = 1; g_aim.car = i;
					g_aim.sit = { sg * 0.97f, 1.42f, zc };
					g_aim.stand = { sg * 0.42f, Y_FLOOR + 0.02f, zc };
					g_aim.yaw = -sg * PI * 0.5f;
					g_aim.prompt = L"[E/右クリック] 座る";
				}
			}
		}


		if ((head || tail) && inCab) {
			const float zs = head ? 1.0f : -1.0f;
			auto bx = [&](float x0, float x1, float y0, float y1, float z0, float z1, float& t) -> bool {
				return rayBox(o, d, { std::min(zs * x0, zs * x1), y0, std::min(zs * z0, zs * z1) }, { std::max(zs * x0, zs * x1), y1, std::max(zs * z0, zs * z1) }, t);
				};
			float t0 = 1e30f, t1 = 1e30f, tc;
			if (bx(-0.90f, -0.30f, 1.115f, 1.50f, 8.10f, 8.55f, tc)) t0 = tc;
			if (bx(-0.88f, -0.32f, 1.50f, 2.30f, 8.06f, 8.15f, tc)) t1 = tc;
			const float t = std::min(t0, t1);
			if (t < best && !blocked(t)) {
				best = t;
				g_aim.valid = true; g_aim.kind = head ? 2 : 3; g_aim.car = i;
				g_aim.sit = { zs * -0.60f, 1.50f, zs * 8.38f };
				g_aim.stand = { zs * -0.60f, 1.12f, zs * 8.88f };
				g_aim.yaw = head ? 0.0f : PI;
				g_aim.prompt = head ? L"[E/右クリック] 運転席に座る" : L"[E/右クリック] 運転台の椅子に座る";
			}
		}
	}


	for (int i = 0; i < NUM_CARS; i += NUM_CARS - 1) {
		if (!fpNearCar(cw[i], eye)) continue;
		const int en = (i == 0) ? 0 : 1, gi = (i == 0) ? 0 : 2;
		const float zs = (i == 0) ? 1.0f : -1.0f;
		const V3 o = xfPoint(inv[i], eye);
		const V3 d = norm(xfPoint(inv[i], eye + dir) - o);
		for (int sd = 0; sd < 2; sd++) {
			const float sg = sd ? 1.0f : -1.0f;
			const int idx = en * 2 + sd;
			const bool open = g_cabTgt[idx] || g_cabPos[idx] > 0.05f;
			const float xa = sg * (open ? 0.60f : 1.25f), xb = sg * 1.55f;
			const float za = zs * (CAB_DOOR_Z0 - 0.05f), zb = zs * (CAB_DOOR_Z1 + 0.05f);
			float t;
			if (!rayBox(o, d, { std::min(xa, xb), Y_FLOOR, std::min(za, zb) }, { std::max(xa, xb), 3.25f, std::max(za, zb) }, t)) continue;
			if (t >= best) continue;
			if (t > 0.03f && g_colCar[gi].rayFirst(o, d, t - 0.03f) < 1e29f) continue;
			best = t;
			g_aim = FPAim(); g_aim.valid = true; g_aim.kind = 20; g_aim.idx = idx; g_aim.car = i;
			g_aim.prompt = g_cabTgt[idx] ? L"[E/右クリック] 乗務員室ドアを閉める" : L"[E/右クリック] 乗務員室ドアを開ける";
		}
	}


	if (g_fp.car < 0) {
		for (const TFrame& B : g_benches) {
			const V3 rel = eye - B.p;
			if (fabsf(rel.x) > 8.0f || fabsf(rel.z) > 8.0f || fabsf(rel.y) > 8.0f) continue;
			const V3 o = { dot(rel, B.r), dot(rel, B.u), dot(rel, B.f) };
			const V3 d = { dot(dir, B.r), dot(dir, B.u), dot(dir, B.f) };
			float t0 = 1e30f, t1 = 1e30f, tc;
			if (rayBox(o, d, { -4.38f, 2.08f, -0.80f }, { -3.82f, 2.16f, 0.80f }, tc)) t0 = tc;
			if (rayBox(o, d, { -4.43f, 2.22f, -0.80f }, { -4.37f, 2.82f, 0.80f }, tc)) t1 = tc;
			const float t = std::min(t0, t1);
			if (t >= best) continue;
			if (g_colWorld.rayFirst(eye, dir, t - 0.03f) < 1e29f) continue;
			best = t;
			const V3 hp = o + d * t;
			const float zc = std::min(0.45f, std::max(-0.45f, hp.z));
			g_aim = FPAim();
			g_aim.valid = true; g_aim.kind = 4; g_aim.car = -1;
			g_aim.sit = B.p + B.r * (-4.12f) + B.u * 2.16f + B.f * zc;
			g_aim.stand = B.p + B.r * (-3.30f) + B.u * 1.69f + B.f * zc;
			g_aim.yaw = atan2f(B.r.x, B.r.z);
			g_aim.prompt = L"[E/右クリック] ベンチに座る";
		}
		if (g_hs.valid) houseAim(eye, dir, best);
	}
}


static void fpUpdate(float dt, bool input, const M4* cw, float dl, float dr, const float* cab, float mdx, float mdy, float trainKmh, V3& eyeOut, V3& atOut) {
	FPState& f = g_fp;
	M4 inv[NUM_CARS];
	for (int i = 0; i < NUM_CARS; i++) inv[i] = inverse4(cw[i]);
	g_fpHitKmh = -1.0f;
	const bool rode = (f.car >= 0) || f.rideGrace > 0.0f;

	if (f.seat == 4 || f.seat == 5) f.pos = f.pos + (f.seatTgt - f.pos) * std::min(1.0f, dt * 12.0f);
	else if (f.seat) f.local = f.local + (f.seatTgt - f.local) * std::min(1.0f, dt * 12.0f);

	if (f.car >= 0) {
		f.pos = xfPoint(cw[f.car], f.local);
		float hd = atan2f(cw[f.car].m[2][0], cw[f.car].m[2][2]);
		if (f.resume) f.heading = hd;
		float d = hd - f.heading;
		while (d > PI) d -= 2.0f * PI;
		while (d < -PI) d += 2.0f * PI;
		f.yaw += d; f.heading = hd;
	}
	f.resume = false;


	if (g_mouseMode) {
		f.yaw += mdx * FP_MOUSE_SENS;
		f.pitch -= mdy * FP_MOUSE_SENS;
	}
	else if (input) {
		const float yawSpeed = 1.5f, pitchSpeed = 1.0f;
		if (GetAsyncKeyState('J') & 0x8000) f.yaw -= yawSpeed * dt;
		if (GetAsyncKeyState('L') & 0x8000) f.yaw += yawSpeed * dt;
		if (GetAsyncKeyState('I') & 0x8000) f.pitch += pitchSpeed * dt;
		if (GetAsyncKeyState('K') & 0x8000) f.pitch -= pitchSpeed * dt;
	}
	f.pitch = std::min(1.45f, std::max(-1.45f, f.pitch));
	while (f.yaw > PI) f.yaw -= 2.0f * PI;
	while (f.yaw < -PI) f.yaw += 2.0f * PI;


	float mf = 0.0f, ms = 0.0f;
	if (input && !f.seat) {
		if (GetAsyncKeyState('W') & 0x8000) mf += 1.0f;
		if (GetAsyncKeyState('S') & 0x8000) mf -= 1.0f;
		if (GetAsyncKeyState('D') & 0x8000) ms += 1.0f;
		if (GetAsyncKeyState('A') & 0x8000) ms -= 1.0f;
	}
	float ml = sqrtf(mf * mf + ms * ms);
	if (ml > 1.0f) { mf /= ml; ms /= ml; }
	const V3 fw = { sinf(f.yaw), 0.0f, cosf(f.yaw) }, rt = { cosf(f.yaw), 0.0f, -sinf(f.yaw) };
	const bool dash = input && (GetAsyncKeyState(VK_SHIFT) & 0x8000);
	const V3 mv = (fw * mf + rt * ms) * ((dash ? FP_DASH : FP_WALK) * dt);
	const float dist = sqrtf(mv.x * mv.x + mv.z * mv.z);
	const int n = std::max(1, (int)ceilf(dist / 0.1f));
	const float dts = dt / n;
	int src = f.car;
	if (f.seat) { f.grounded = true; f.vy = 0.0f; }
	else {
		if (input && f.grounded && (GetAsyncKeyState(VK_SPACE) & 0x8000)) {
			f.vy = FP_JUMP; f.grounded = false;
			f.airVel = (f.car >= 0) ? V3{ cw[f.car].m[2][0] * g_trainV, 0.0f, cw[f.car].m[2][2] * g_trainV } : V3{ 0, 0, 0 };
		}
		for (int k = 0; k < n; k++) {
			f.pos.x += mv.x / n; f.pos.z += mv.z / n;
			if (!f.grounded) { f.pos.x += f.airVel.x * dts; f.pos.z += f.airVel.z * dts; }
			fpCollide(cw, inv, dl, dr, cab);
			fpGround(dts, cw, inv, src);
			if (f.grounded) f.airVel = { 0, 0, 0 };
		}


		if (!rode && trainKmh >= HIT_MIN_KMH) {
			for (int i = 0; i < NUM_CARS; i++) {
				if (!fpNearCar(cw[i], f.pos)) continue;
				const V3 lc = xfPoint(inv[i], f.pos);
				if (fabsf(lc.x) > 1.40f + FP_R - 0.03f || fabsf(lc.z) > 10.0f + FP_R * 0.5f) continue;
				if (lc.y > 3.95f || lc.y + 1.75f < 0.55f) continue;
				const float sx = (lc.x >= 0.0f) ? 1.0f : -1.0f;
				const float push = sx * (1.40f + FP_R + 0.12f) - lc.x;
				f.pos.x += push * cw[i].m[0][0]; f.pos.z += push * cw[i].m[0][2];
				f.vy = 3.5f; f.grounded = false;
				g_fpHitKmh = trainKmh;
				break;
			}
		}
	}

	if (f.pos.y < -150.0f) { f.pos.y += 200.0f; f.vy = 0.0f; }


	if (f.grounded && src >= 0) {
		f.car = src; f.local = xfPoint(inv[src], f.pos);
		f.heading = atan2f(cw[src].m[2][0], cw[src].m[2][2]);
	}
	else f.car = -1;
	if (f.car >= 0) f.rideGrace = 0.7f; else f.rideGrace = std::max(0.0f, f.rideGrace - dt);

	float dy = f.pos.y - f.eyeY;
	if (fabsf(dy) > 3.0f) f.eyeY = f.pos.y; else f.eyeY += dy * std::min(1.0f, dt * 20.0f);
	eyeOut = { f.pos.x, f.eyeY + (f.seat == 5 ? FP_EYE_LIE : (f.seat ? FP_EYE_SIT : FP_EYE)), f.pos.z };
	V3 dir = { sinf(f.yaw) * cosf(f.pitch), sinf(f.pitch), cosf(f.yaw) * cosf(f.pitch) };
	atOut = eyeOut + dir * 10.0f;

	if (input && !f.seat) fpAim(cw, inv, dl, dr, cab, eyeOut, dir);
	else g_aim = FPAim();
}


static int fpInteract(const M4* cw) {
	FPState& f = g_fp;
	M4 inv[NUM_CARS];
	for (int i = 0; i < NUM_CARS; i++) inv[i] = inverse4(cw[i]);
	if (f.seat == 4 || f.seat == 5) {
		f.pos = f.standLocal; f.car = -1; f.vy = 0.0f; f.grounded = false; f.seat = 0; f.seatCar = -1; f.resume = false;
		g_driverSeat = false;
		return 3;
	}
	if (f.seat) {
		const int car = f.seatCar;
		const float hd = atan2f(cw[car].m[2][0], cw[car].m[2][2]);
		if (f.seat == 2) { f.yaw = hd + f.seatYaw; f.pitch = 0.0f; }
		f.local = f.standLocal; f.car = car; f.heading = hd;
		f.pos = xfPoint(cw[car], f.local);
		f.vy = 0.0f; f.grounded = false; f.seat = 0; f.resume = false;
		g_driverSeat = false;
		return 3;
	}
	if (!g_aim.valid) return 0;
	if (g_aim.kind >= 10) { g_act = g_aim; g_aim = FPAim(); return 4; }
	if (g_aim.kind == 4 || g_aim.kind == 5 || g_aim.kind == 6) {
		f.car = -1; f.grounded = true; f.vy = 0.0f;
		f.seat = (g_aim.kind == 6) ? 5 : 4; f.seatCar = -1; f.seatTgt = g_aim.sit; f.standLocal = g_aim.stand; f.seatYaw = 0.0f;
		f.yaw = g_aim.yaw; f.pitch = (g_aim.kind == 6) ? 0.55f : 0.0f; f.resume = false;
		g_aim = FPAim();
		return 1;
	}
	const int car = g_aim.car;
	if (f.car >= 0) f.pos = xfPoint(cw[f.car], f.local);
	const float hd = atan2f(cw[car].m[2][0], cw[car].m[2][2]);
	f.local = xfPoint(inv[car], f.pos);
	f.car = car; f.heading = hd; f.grounded = true; f.vy = 0.0f;
	f.seat = g_aim.kind; f.seatCar = car; f.seatTgt = g_aim.sit; f.standLocal = g_aim.stand; f.seatYaw = g_aim.yaw;
	f.yaw = hd + g_aim.yaw; f.pitch = 0.0f; f.resume = false;
	g_aim = FPAim();
	if (f.seat == 2) { g_driverSeat = true; return 2; }
	return 1;
}


static V3 trainPos(const Train& T, float s) { const Pose P = trackAt(s); return P.p + rightOf(P.f) * pathLat(T.path, s); }
static void trainCars(const Train& T, M4* out) {
	for (int i = 0; i < NUM_CARS; i++) {
		const float sc = wrapS(T.s - (float)T.dir * (CAR_LEN * 0.5f + i * (CAR_LEN + CAR_GAP)));
		const V3 pf = trainPos(T, sc + (float)T.dir * BOGIE_HALF), pr = trainPos(T, sc - (float)T.dir * BOGIE_HALF);
		const V3 fwd = norm(pf - pr), ctr = (pf + pr) * 0.5f + V3{ 0, CAR_Y, 0 };
		out[i] = basisMat(rightOf(fwd), { 0, 1, 0 }, fwd, ctr);
	}
}

struct DynMesh { ID3D12Resource* res = nullptr; Vtx* ptr = nullptr; UINT cap = 0; Mesh m; };
static DynMesh makeDyn(UINT capVerts) {
	DynMesh d; d.cap = capVerts;
	d.res = createBuf((UINT64)capVerts * sizeof(Vtx), D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ);
	d.res->Map(0, nullptr, (void**)&d.ptr);
	d.m.res = d.res; d.m.vbv.BufferLocation = d.res->GetGPUVirtualAddress(); d.m.vbv.SizeInBytes = capVerts * (UINT)sizeof(Vtx); d.m.vbv.StrideInBytes = (UINT)sizeof(Vtx); d.m.start = 0; d.m.n = 0;
	return d;
}
static void dynSet(DynMesh& d, const Verts& v) { const UINT n = (UINT)std::min<size_t>(v.size(), d.cap); if (n) memcpy(d.ptr, v.data(), n * sizeof(Vtx)); d.m.n = n; }


static const int LED_W = 256, LED_H = 80;
static const float LED_PITCH = 0.0185f;
struct LedBoard { DynMesh dm; bool made = false; unsigned hash = 0; };
static std::vector<LedBoard> g_led;
static HDC g_ledDC = nullptr; static HBITMAP g_ledBmp = nullptr; static DWORD* g_ledBits = nullptr; static HFONT g_ledFont = nullptr;
static void ledInit() {
	HDC sdc = GetDC(nullptr); g_ledDC = CreateCompatibleDC(sdc); ReleaseDC(nullptr, sdc);
	BITMAPINFO bi{}; bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER); bi.bmiHeader.biWidth = LED_W; bi.bmiHeader.biHeight = -LED_H;
	bi.bmiHeader.biPlanes = 1; bi.bmiHeader.biBitCount = 32; bi.bmiHeader.biCompression = BI_RGB;
	g_ledBmp = CreateDIBSection(g_ledDC, &bi, DIB_RGB_COLORS, (void**)&g_ledBits, nullptr, 0);
	if (!g_ledBmp || !g_ledBits) { g_led.resize(g_boardObjs.size()); return; }   // 失敗時はLEDを表示しない
	SelectObject(g_ledDC, g_ledBmp);
	g_ledFont = CreateFontW(16, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE, DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, NONANTIALIASED_QUALITY, FIXED_PITCH | FF_MODERN, L"MS Gothic");
	SelectObject(g_ledDC, g_ledFont);
	SetBkMode(g_ledDC, OPAQUE); SetBkColor(g_ledDC, RGB(0, 0, 0)); SetTextColor(g_ledDC, RGB(255, 255, 255));
	g_led.resize(g_boardObjs.size());
}
static std::wstring hhmm(double secOfDay) {
	int m = (int)(fmod(secOfDay, 86400.0) / 60.0); wchar_t b[16]; swprintf_s(b, L"%02d:%02d", (m / 60) % 24, m % 60); return b;
}
static std::wstring destName(int stn) {
	if (stn == 0) return L"周回";
	std::wstring n = g_stations[0].name; if (n.size() > 4) n = n.substr(0, 4);
	return n + L"ゆき";
}
static void boardText(int line, int stn, std::wstring txt[4]) {
	BoardRow rows[4]; boardRows(line, stn, rows);
	const wchar_t* lab[4] = { L"停車中", L"先発", L"次発", L"次次発" };
	for (int i = 0; i < 4; i++) {
		if (rows[i].kind == 0) { txt[i] = (i == 0) ? L"停車中  なし" : std::wstring(lab[i]) + L" --:--"; continue; }
		if (rows[i].kind == 3) { txt[i] = L"停車中 終着 降車のみ"; continue; }
		txt[i] = std::wstring(lab[i]) + L" " + hhmm(rows[i].dep) + L" " + TYPE_NAME[rows[i].type] + L" " + destName(stn);
	}
	if (rows[1].kind == 0 && rows[2].kind == 0 && rows[3].kind == 0) txt[2] = L"本日の運転は終了しました";
}
static void ledUpdate(int bi) {
	if (!g_ledBits) return;
	const BoardObj& B = g_boardObjs[bi]; LedBoard& L = g_led[bi];
	std::wstring txt[4]; boardText(B.line, B.stn, txt);
	unsigned h = 2166136261u; for (int i = 0; i < 4; i++) for (wchar_t c : txt[i]) { h ^= (unsigned)c; h *= 16777619u; }
	if (L.made && h == L.hash) return;
	L.hash = h;
	memset(g_ledBits, 0, (size_t)LED_W * LED_H * 4);
	for (int r = 0; r < 4; r++) TextOutW(g_ledDC, 4, 2 + r * 19, txt[r].c_str(), (int)txt[r].size());
	GdiFlush();
	const V3 rowCol[4] = { { 0.35f, 1.0f, 0.45f }, { 1.0f, 0.72f, 0.12f }, { 1.0f, 0.72f, 0.12f }, { 1.0f, 0.72f, 0.12f } };
	Verts v; v.reserve(8000);
	const V3 up = B.u;
	for (int y = 0; y < LED_H; y++) {
		const int r = std::min(3, std::max(0, (y - 2) / 19));
		int x = 0;
		while (x < LED_W) {
			if ((g_ledBits[y * LED_W + x] & 0xFF) < 100) { x++; continue; }
			int x1 = x; while (x1 + 1 < LED_W && (g_ledBits[y * LED_W + x1 + 1] & 0xFF) >= 100) x1++;
			const float u0 = (x - LED_W * 0.5f) * LED_PITCH, u1 = (x1 + 1 - LED_W * 0.5f) * LED_PITCH;
			const float yt = (LED_H * 0.5f - y) * LED_PITCH, yb = yt - LED_PITCH * 0.86f;
			for (int face = 0; face < 2; face++) {
				const float sg = face ? -1.0f : 1.0f;
				const float la = (face ? u0 : -u0), lb = (face ? u1 : -u1);
				const V3 n = B.f * sg, o = B.c + B.f * (0.105f * sg);
				const V3 p0 = o + B.r * la + up * yt, p1 = o + B.r * lb + up * yt, p2 = o + B.r * lb + up * yb, p3 = o + B.r * la + up * yb;
				const V3 c = rowCol[r];
				const V3 pp[6] = { p0, p1, p2, p0, p2, p3 };
				for (int i = 0; i < 6; i++) v.push_back({ pp[i], n, c, 2.0f });
			}
			x = x1 + 1;
		}
	}
	if (!L.made || (UINT)v.size() > L.dm.cap) {
		if (L.made && L.dm.res) { L.dm.res->Unmap(0, nullptr); L.dm.res->Release(); }
		L.dm = makeDyn((UINT)v.size() + 2048); L.made = true;
	}
	dynSet(L.dm, v);
}


static DynMesh g_dynLamp;
static Mesh g_armMesh;
static void buildLampVerts(Verts& v, V3 eye, double timeAcc) {
	v.clear();
	auto lamp = [&](V3 c, V3 face, V3 col, float hs) {
		const V3 u = { 0, 1, 0 }; V3 r = norm(cross(u, face));
		const V3 pp[6] = { c - r * hs - u * hs, c + r * hs - u * hs, c + r * hs + u * hs, c - r * hs - u * hs, c + r * hs + u * hs, c - r * hs + u * hs };
		for (int i = 0; i < 6; i++) v.push_back({ pp[i], face, col, 2.0f });
		};
	for (const SigObj& so : g_sigObjs) {
		const V3 d = so.head - eye; if (d.x * d.x + d.y * d.y + d.z * d.z > 400.0f * 400.0f) continue;
		const int a = g_rs.aspect[so.trk][so.k];
		const V3 base = so.head + so.face * 0.14f;
		if (a == 0) lamp(base + V3{ 0, 0.40f, 0 }, so.face, { 1.0f, 0.10f, 0.08f }, 0.14f);
		if (a == 1 || a == 2) lamp(base, so.face, { 1.0f, 0.80f, 0.10f }, 0.14f);
		if (a == 2 || a == 3) lamp(base - V3{ 0, 0.40f, 0 }, so.face, { 0.15f, 1.0f, 0.35f }, 0.14f);
	}
	const int ph = ((int)(fmod(timeAcc, 1.0) * 2.0)) & 1;
	for (size_t xi = 0; xi < g_xingObjs.size(); xi++) {
		if (xi >= g_rs.cx.size() || !g_rs.cx[xi].warn) continue;
		const TFrame& F = g_xingObjs[xi].F;
		const V3 d = F.p - eye; if (d.x * d.x + d.z * d.z > 400.0f * 400.0f) continue;
		for (int sg = -1; sg <= 1; sg += 2) for (int sz = -1; sz <= 1; sz += 2) for (int k = -1; k <= 1; k += 2) {
			if (((k > 0) ? 1 : 0) != ph) continue;
			const V3 c = fPt(F, sg * 12.6f + k * 0.32f, 3.55f, 3.9f * sz) + F.r * (0.13f * sg);
			lamp(c, F.r * (float)sg, { 1.0f, 0.08f, 0.06f }, 0.11f);
		}
	}
}
static Verts buildArmVerts() {
	Verts v;
	for (int i = 0; i < 9; i++) addBox(v, { 0, 0, 0.2f + i * 0.4f }, { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.035f, 0.06f, 0.2f }, (i & 1) ? V3{ 0.95f, 0.95f, 0.95f } : V3{ 0.90f, 0.12f, 0.10f });
	return v;
}
static M4 armMatrix(const TFrame& F, int sg, int sz, float arm) {
	const float a = (1.0f - arm) * (PI * 0.5f);
	const V3 d0 = F.f * (float)(-sz), dir = norm(d0 * cosf(a) + F.u * sinf(a));
	const V3 x = F.r, y = cross(dir, x);
	return basisMat(x, y, dir, fPt(F, sg * 12.6f, 1.0f, 3.9f * sz));
}


static void showNextTrains(int mi, std::vector<std::wstring>& out) {
	out.clear();
	const StopMarker& M = g_markers[mi];
	BoardRow rows[4]; boardRows(M.line, M.stn, rows);
	const double now = fmod(g_rs.clock, 86400.0);
	wchar_t b[128];
	swprintf_s(b, L"%s駅  %s", g_stations[M.stn].name.c_str(), M.line == 0 ? L"下り線" : L"上り線");
	out.push_back(b);
	if (rows[0].kind == 3) out.push_back(L"停車中: 終着（降車のみ）");
	else if (rows[0].kind == 1) {
		const int m = std::max(0, (int)ceil((rows[0].dep - now) / 60.0));
		swprintf_s(b, L"停車中: %s %s  あと%d分で発車", TYPE_NAME[rows[0].type], destName(M.stn).c_str(), m);
		out.push_back(b);
	}
	int shown = 0;
	for (int i = 1; i < 4 && shown < 2; i++) if (rows[i].kind == 2) {
		const int m = std::max(0, (int)ceil((rows[i].dep - now) / 60.0));
		swprintf_s(b, L"%s: %d分後  (%s発 %s)", shown == 0 ? L"次の電車" : L"その次", m, hhmm(rows[i].dep).c_str(), TYPE_NAME[rows[i].type]);
		out.push_back(b); shown++;
	}
	if (!shown) out.push_back(L"本日の運転は終了しました");
}

int WINAPI wWinMain(_In_ HINSTANCE hi, _In_opt_ HINSTANCE, _In_ PWSTR, _In_ int) {
	SetProcessDPIAware();
	WNDCLASSEXW wc{ sizeof(wc) };
	wc.lpfnWndProc = WndProc; wc.hInstance = hi; wc.hCursor = LoadCursor(nullptr, IDC_ARROW); wc.lpszClassName = L"TrainSimWnd";
	wc.hIcon = LoadIconW(hi, MAKEINTRESOURCEW(IDI_ICON1));
	wc.hIconSm = (HICON)LoadImageW(
		hi,
		MAKEINTRESOURCEW(IDI_ICON1),
		IMAGE_ICON,
		GetSystemMetrics(SM_CXSMICON),
		GetSystemMetrics(SM_CYSMICON),
		LR_DEFAULTCOLOR
	);
	RegisterClassExW(&wc);
	RECT rc = { 0, 0, 1280, 720 }; AdjustWindowRect(&rc, WS_OVERLAPPEDWINDOW, FALSE);
	HWND hwnd = CreateWindowW(L"TrainSimWnd", L"SilkHat Game series.  Now Loading... ", WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, CW_USEDEFAULT,
		rc.right - rc.left, rc.bottom - rc.top, nullptr, nullptr, hi, nullptr);
	if (!hwnd || !initD3D(hwnd)) {
		std::string m = "Direct3D 12 の初期化に失敗しました\n(Direct3D 12 対応GPU/ドライバーが必要です)\n\n" + g_shaderErr.substr(0, 1800);
		MessageBoxA(nullptr, m.c_str(), "TrainSim", MB_OK); return 1;
	}
	SetWindowTextW(hwnd, L"SilkHat - Loading ...");

	generateProceduralWorld();
	railLayoutInit(); railInit();

	{

		const float TARGET = 0.39f;
		float mx = 0.0f;
		for (float s = 0.0f; s < g_totalTrackLen; s += 2.0f) {
			float g = fabsf((trackHeight(s) - trackHeight(s - TRAIN_LEN)) / TRAIN_LEN);
			mx = std::max(mx, g / sqrtf(1.0f + g * g));
		}
		g_gravScale = (mx > 1e-4f) ? std::min(1.0f, TARGET / (9.80665f * mx)) : 1.0f;
	}


	Verts vTrack = buildTrack();
	Verts vScen = buildScenery();
	const UINT nTrack = (UINT)vTrack.size(), nScen = (UINT)vScen.size();
	Verts vWorld; vWorld.reserve((size_t)nTrack + nScen);
	vWorld.insert(vWorld.end(), vTrack.begin(), vTrack.end());
	vWorld.insert(vWorld.end(), vScen.begin(), vScen.end());
	Verts().swap(vTrack); Verts().swap(vScen);
	buildBenchList();
	const size_t nWorldVerts = vWorld.size();
	vWorld.insert(vWorld.end(), g_colExtra.begin(), g_colExtra.end());
	buildCollision(vWorld);
	vWorld.resize(nWorldVerts);
	if (g_hs.valid) for (int d = 0; d < NHD; d++) { Verts dw = hzDoorWorld(d, 0.0f); vWorld.insert(vWorld.end(), dw.begin(), dw.end()); }
	Mesh world = makeMesh(vWorld);
	Mesh track = subMesh(world, 0, nTrack);
	Mesh scenery = subMesh(world, nTrack, nScen);
	Mesh bldInter = makeMesh(g_bldInter), bldGlass = makeMesh(g_bldGlass);
	g_armMesh = makeMesh(buildArmVerts()); g_dynLamp = makeDyn(12000); ledInit();
	Mesh bodyMid = makeMesh(buildCarBody(false, false));
	Mesh bodyHead = makeMesh(buildCarBody(true, false));
	Mesh bodyTail = makeMesh(buildCarBody(false, true));
	Mesh doorLeafL = makeMesh(buildDoorLeaf(-1.0f));
	Mesh doorLeafR = makeMesh(buildDoorLeaf(1.0f));
	Mesh glassMid = makeMesh(buildCarGlass(false, false));
	Mesh glassHead = makeMesh(buildCarGlass(true, false));
	Mesh glassTail = makeMesh(buildCarGlass(false, true));
	Mesh doorGlassL = makeMesh(buildDoorGlass(-1.0f));
	Mesh doorGlassR = makeMesh(buildDoorGlass(1.0f));
	Mesh windF = makeMesh(buildWindscreen(1.0f));
	Mesh windR = makeMesh(buildWindscreen(-1.0f));
	if (g_hs.valid) {
		for (int d = 0; d < NHD; d++) g_hm.door[d] = makeMesh(g_hvx.door[d]);
		g_hm.glass = makeMesh(g_hvx.glass);
		for (int k = 0; k < 3; k++) g_hm.glow[k] = makeMesh(g_hvx.glow[k]);
		g_hm.flame = makeMesh(g_hvx.flame); g_hm.sphere = makeMesh(g_hvx.sphere);
		g_hm.water = makeMesh(g_hvx.water); g_hm.pcStatic = makeMesh(g_hvx.pcStatic); g_hm.pcNeedle = makeMesh(g_hvx.pcNeedle); g_hm.pcFill = makeMesh(g_hvx.pcFill);
		for (int k = 0; k < 6; k++) g_hm.pcBar[k] = makeMesh(g_hvx.pcBar[k]);
	}
	g_curE = 2.0f;
	Mesh mcHandle = makeMesh(buildMcHandle());
	Mesh needle = makeMesh(buildNeedle());
	Mesh lampOn = makeMesh(buildDoorLamp({ 0.20f, 1.00f, 0.30f }));
	Mesh lampOff = makeMesh(buildDoorLamp({ 0.05f, 0.20f, 0.08f }));
	g_curE = 0.0f;

	Mesh cabDoorM[2][2], cabGlassM[2][2];
	for (int s = 0; s < 2; s++) for (int e = 0; e < 2; e++) {
		cabDoorM[s][e] = makeMesh(buildCabDoor(s ? 1.0f : -1.0f, e ? -1.0f : 1.0f));
		cabGlassM[s][e] = makeMesh(buildCabDoorGlass(s ? 1.0f : -1.0f, e ? -1.0f : 1.0f));
	}


	Mesh rtMesh[3];
	// 車両の代理メッシュ（遠くの車両の描画にも使う。軽い）。BVH/BLAS/TLAS の作成は重いので、レイトレーシングをオンにした時まで行わない
	Verts rtV[3] = { buildCarBody(true, false), buildCarBody(false, false), buildCarBody(false, true) };
	for (int k = 0; k < 3; k++) appendLeaves(rtV[k]);
	appendCabDoors(rtV[0], 1.0f); appendCabDoors(rtV[2], -1.0f);
	for (int k = 0; k < 3; k++) rtMesh[k] = makeMesh(rtV[k]);
	bool rtReady = false;   // レイトレーシング用データを作成済みか（グラフィックレベルが「オフ」のままなら作らない）
	{   // 未作成の間も、ディスクリプタテーブルの中身が空にならないようヌル記述子を置いておく
		D3D12_SHADER_RESOURCE_VIEW_DESC nb{}; nb.Format = DXGI_FORMAT_UNKNOWN; nb.ViewDimension = D3D12_SRV_DIMENSION_BUFFER;
		nb.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING; nb.Buffer.StructureByteStride = 16; nb.Buffer.NumElements = 1;
		g_dev->CreateShaderResourceView(nullptr, &nb, srvCpu(0));
		g_dev->CreateShaderResourceView(nullptr, &nb, srvCpu(1));
		D3D12_SHADER_RESOURCE_VIEW_DESC nd{}; nd.Format = DXGI_FORMAT_R8G8B8A8_UNORM; nd.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
		nd.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING; nd.Texture2D.MipLevels = 1;
		g_dev->CreateShaderResourceView(nullptr, &nd, srvCpu(2));
	}
	flushInit();
	ID3D12Resource* hvbUp = createBuf((UINT64)HV_MAX * sizeof(HV), D3D12_HEAP_TYPE_UPLOAD, D3D12_RESOURCE_STATE_GENERIC_READ);
	UINT8* hvbPtr = nullptr; hvbUp->Map(0, nullptr, (void**)&hvbPtr);

	railSync(g_tod);
	const size_t NTR = g_rs.tr.size();
	std::vector<M4> trCW(NTR * NUM_CARS); std::vector<char> nearTr(NTR, 0);
	int focusTr = -1;
	float regenWait = 0.0f, hpAcc = 0.0f;
	int   hp = 1000; float hurtCool = 0.0f, hurtFlash = 0.0f;
	std::vector<std::wstring> infoLines; float infoT = 0.0f;
	Verts lampV; lampV.reserve(4000);
	float mcAng = 0.0f;

	std::wstring toast; float toastT = 0;
	auto setToast = [&](const wchar_t* s, float t) { toast = s; toastT = t; };

	LARGE_INTEGER fq, t0; QueryPerformanceFrequency(&fq); QueryPerformanceCounter(&t0);
	double timeAcc = 0.0;
	float fpsAcc = 0; int fpsFrames = 0;
	const wchar_t* camNames[6] = { L"", L"追従 (1)", L"", L"", L"", L"一人称視点 (2)" };
	float* cabPos = g_cabPos; bool* cabTarget = g_cabTgt;
	M4 prevVp = ident(); V3 prevEye = { 0, 0, 0 }; int prevCam = g_cam; bool prevValid = false;

	bool sleepOn = false; float sleepT = 0.0f; double sleepStart = 0.0, sleepSpan = 0.0;
	bool mouseCaptured = false, mouseSkip = false;
	MSG msg{};
	startShaderLoad();   // 起動後、ゲームを止めずにバックグラウンドで重いシェーダーを読み込む
	while (!g_quit) {
		while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) { TranslateMessage(&msg); DispatchMessageW(&msg); if (msg.message == WM_QUIT) g_quit = true; }
		if (g_quit) break;

		LARGE_INTEGER t1; QueryPerformanceCounter(&t1);
		float dt = std::min(0.05f, (float)(t1.QuadPart - t0.QuadPart) / fq.QuadPart); t0 = t1;

		if (g_pause) dt = 0.0f;
		timeAcc += dt;

		if (g_pause) { g_interactReq = false; g_atoReq = false; g_notchResetReq = false; }
		if (g_qToast) { setToast(g_qToast, g_qToastT); g_qToast = nullptr; }
		if (g_respawnReq) {
			g_respawnReq = false;
			if (g_dead) {
				g_dead = false; hp = 1000; hurtCool = 2.0f; hurtFlash = 0.0f; g_respHover = false;
				g_driverSeat = false; g_cam = 5;
				fpSpawn(0.0f, 0);
				setToast(L"リスポーンしました", 2.5f);
			}
		}


		for (Train& tr : g_rs.tr) tr.onboard = false;
		if (focusTr >= 0 && g_fp.valid && (g_fp.car >= 0 || (g_fp.seat > 0 && g_fp.seatCar >= 0))) g_rs.tr[focusTr].onboard = true;
		railSync(g_tod);
		toastT = std::max(0.0f, toastT - dt);
		infoT = std::max(0.0f, infoT - dt);


		for (Train& tr : g_rs.tr) for (int s = 0; s < 4; s++) tr.cabPos[s] = std::min(1.0f, std::max(0.0f, tr.cabPos[s] + (tr.cabTgt[s] ? 1.0f : -1.0f) * dt / 1.2f));


		if (g_cam != 5 && !(g_cam == 1 && g_mouseMode) && !g_pause && GetForegroundWindow() == hwnd) {
			const float yawSpeed = 1.5f, pitchSpeed = 1.0f;
			if (GetAsyncKeyState('J') & 0x8000) g_camYaw -= yawSpeed * dt;
			if (GetAsyncKeyState('L') & 0x8000) g_camYaw += yawSpeed * dt;
			if (GetAsyncKeyState('I') & 0x8000) g_camPitch = std::min(1.2f, g_camPitch + pitchSpeed * dt);
			if (GetAsyncKeyState('K') & 0x8000) g_camPitch = std::max(-0.4f, g_camPitch - pitchSpeed * dt);
		}

		float fpMdx = 0.0f, fpMdy = 0.0f;
		{
			const bool want = ((g_cam == 5 || g_cam == 1) && g_mouseMode && !g_pause && !g_dead && GetForegroundWindow() == hwnd);
			RECT cr; GetClientRect(hwnd, &cr); POINT tl = { 0, 0 }; ClientToScreen(hwnd, &tl);
			const int ccx = tl.x + cr.right / 2, ccy = tl.y + cr.bottom / 2;
			if (want != mouseCaptured) {
				if (want) { RECT sr = { tl.x, tl.y, tl.x + cr.right, tl.y + cr.bottom }; ClipCursor(&sr); while (ShowCursor(FALSE) >= 0) {} SetCursorPos(ccx, ccy); }
				else { ClipCursor(nullptr); while (ShowCursor(TRUE) < 0) {} }
				mouseCaptured = want; mouseSkip = true;
			}
			if (mouseCaptured) {
				POINT p; GetCursorPos(&p);
				if (!mouseSkip) { fpMdx = (float)(p.x - ccx); fpMdy = (float)(p.y - ccy); }
				mouseSkip = false;
				SetCursorPos(ccx, ccy);
				if (g_cam == 1) {
					g_camYaw += fpMdx * FP_MOUSE_SENS;
					g_camPitch = std::clamp(g_camPitch - fpMdy * FP_MOUSE_SENS, -0.4f, 1.2f);
					fpMdx = fpMdy = 0.0f;
				}
			}
		}

		const V3 refPos = g_fp.valid ? g_fp.pos : prevEye;
		for (size_t ti = 0; ti < NTR; ti++) {
			const Train& T = g_rs.tr[ti];
			const V3 h = trainPos(T, T.s), t2 = trainPos(T, wrapS(T.s - (float)T.dir * TRAIN_LEN));
			const float d1 = (h.x - refPos.x) * (h.x - refPos.x) + (h.z - refPos.z) * (h.z - refPos.z), d2 = (t2.x - refPos.x) * (t2.x - refPos.x) + (t2.z - refPos.z) * (t2.z - refPos.z);
			nearTr[ti] = (std::min(d1, d2) < 1300.0f * 1300.0f) || ((int)ti == focusTr);
			if (nearTr[ti]) trainCars(T, &trCW[ti * NUM_CARS]);
		}
		{
			const bool riding = g_fp.valid && (g_fp.car >= 0 || (g_fp.seat > 0 && g_fp.seatCar >= 0));
			if (!(riding && focusTr >= 0)) {
				focusTr = -1; float bd = 45.0f * 45.0f;
				if (g_fp.valid) for (size_t ti = 0; ti < NTR; ti++) {
					if (!nearTr[ti]) continue;
					for (int i = 0; i < NUM_CARS; i++) {
						const M4& cm = trCW[ti * NUM_CARS + i];
						const float dd = (cm.m[3][0] - g_fp.pos.x) * (cm.m[3][0] - g_fp.pos.x) + (cm.m[3][1] - g_fp.pos.y) * (cm.m[3][1] - g_fp.pos.y) + (cm.m[3][2] - g_fp.pos.z) * (cm.m[3][2] - g_fp.pos.z);
						if (dd < bd) { bd = dd; focusTr = (int)ti; }
					}
				}
			}
		}
		M4 cwf[NUM_CARS]; float focusDL = 0.0f, focusDR = 0.0f, focusKmh = 0.0f;
		if (focusTr >= 0) {
			Train& FT = g_rs.tr[focusTr];
			for (int i = 0; i < NUM_CARS; i++) cwf[i] = trCW[(size_t)focusTr * NUM_CARS + i];
			focusDL = FT.doorL; focusDR = FT.doorR; focusKmh = FT.v * 3.6f; g_trainV = FT.v;
			for (int s = 0; s < 4; s++) { g_cabPos[s] = FT.cabPos[s]; g_cabTgt[s] = FT.cabTgt[s]; }
		}
		else {
			for (int i = 0; i < NUM_CARS; i++) cwf[i] = basisMat({ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0.0f, -5000.0f, 0.0f });
			g_trainV = 0.0f; for (int s = 0; s < 4; s++) { g_cabPos[s] = 0.0f; g_cabTgt[s] = false; }
		}
		if (g_hs.valid) houseUpdate(dt);
		if (!g_fp.valid) fpSpawn(0.0f);
		if (g_cam != 5) g_aim = FPAim();
		if (g_interactReq) {
			g_interactReq = false;
			if (!g_dead) {
				const int r = fpInteract(cwf);
				if (r == 2) setToast(L"運転席に座りました（見学のみ。1:追従 2:一人称視点 / E:席を立つ）", 3.5f);
				else if (r == 1) setToast(L"座りました（Eキーで立ち上がる）", 2.0f);
				else if (r == 3) g_cam = 5;
				else if (r == 4) {
					if (g_act.kind == 10) g_hs.doorTgt[g_act.idx] = !g_hs.doorTgt[g_act.idx];
					else if (g_act.kind == 11) g_hs.roomOn[g_act.idx] = !g_hs.roomOn[g_act.idx];
					else if (g_act.kind == 12) { g_hs.stoveOn = !g_hs.stoveOn; setToast(g_hs.stoveOn ? L"コンロに火をつけました" : L"火を消しました", 1.8f); }
					else if (g_act.kind == 13) g_hs.lampOn = !g_hs.lampOn;
					else if (g_act.kind == 14) g_hs.tapOn = !g_hs.tapOn;
					else if (g_act.kind == 20) cabTarget[g_act.idx] = !cabTarget[g_act.idx];
					else if (g_act.kind == 15) g_hs.pcOn = !g_hs.pcOn;
					else if (g_act.kind == 30) { showNextTrains(g_act.idx, infoLines); infoT = 9.0f; }
				}
			}
		}
		if (g_cam == 5 && prevCam != 5 && g_fp.seat == 2 && g_fp.seatCar >= 0) {
			const float hd = atan2f(cwf[g_fp.seatCar].m[2][0], cwf[g_fp.seatCar].m[2][2]);
			g_fp.yaw = hd + g_fp.seatYaw; g_fp.pitch = 0.0f; g_fp.heading = hd; g_fp.resume = true;
		}
		V3 eye, at;
		switch (g_cam) {
		case 1: {
			const Train& CT = g_rs.tr[focusTr >= 0 ? focusTr : 0];
			const V3 pcp = trainPos(CT, wrapS(CT.s - (float)CT.dir * TRAIN_LEN * 0.5f)); Pose pc; pc.p = pcp;
			V3 targetCenter = pc.p + V3{ 0, 2.0f, 0 };
			float dist = 32.0f;
			float yaw = g_camYaw;
			float pitch = std::clamp(g_camPitch, -0.4f, 1.2f);
			eye = targetCenter + V3{ sinf(yaw) * cosf(pitch) * dist, sinf(pitch) * dist + 6.0f, -cosf(yaw) * cosf(pitch) * dist };
			eye.y = std::max(eye.y, terrainH(eye.x, eye.z) + 1.5f);
			at = targetCenter;
			break;
		}
		default: {
			fpUpdate(dt, !g_pause && !g_dead && GetForegroundWindow() == hwnd, cwf, focusDL, focusDR, g_cabPos, fpMdx, fpMdy, g_dead ? 0.0f : focusKmh, eye, at);
			break;
		}
		}
		if (focusTr >= 0) for (int s = 0; s < 4; s++) g_rs.tr[focusTr].cabTgt[s] = g_cabTgt[s];
		if (g_cam == 5 && g_fp.valid && !g_dead && g_fp.car < 0) {
			const bool rode2 = g_fp.rideGrace > 0.0f;
			for (size_t ti = 0; ti < NTR && !rode2; ti++) {
				if ((int)ti == focusTr || !nearTr[ti] || g_rs.tr[ti].v * 3.6f < HIT_MIN_KMH) continue;
				for (int i = 0; i < NUM_CARS; i++) {
					const M4& cm = trCW[ti * NUM_CARS + i];
					if (!fpNearCar(cm, g_fp.pos)) continue;
					const V3 lc = xfPoint(inverse4(cm), g_fp.pos);
					if (fabsf(lc.x) > 1.40f + FP_R - 0.03f || fabsf(lc.z) > 10.0f + FP_R * 0.5f || lc.y > 3.95f || lc.y + 1.75f < 0.55f) continue;
					const float sx = (lc.x >= 0.0f) ? 1.0f : -1.0f, push = sx * (1.40f + FP_R + 0.12f) - lc.x;
					g_fp.pos.x += push * cm.m[0][0]; g_fp.pos.z += push * cm.m[0][2]; g_fp.vy = 3.5f; g_fp.grounded = false;
					g_fpHitKmh = std::max(g_fpHitKmh, g_rs.tr[ti].v * 3.6f);
					break;
				}
			}
			if (!g_fp.seat && !g_aim.valid) {
				const V3 dv = norm(at - eye); int best = -1; float bt = 5.0f;
				for (size_t mi = 0; mi < g_markers.size(); mi++) {
					const V3 oc = g_markers[mi].pos - eye; const float tproj = dot(oc, dv); if (tproj < 0.2f || tproj > bt) continue;
					const V3 cp = eye + dv * tproj - g_markers[mi].pos;
					if (dot(cp, cp) < 0.85f * 0.85f) { best = (int)mi; bt = tproj; }
				}
				if (best >= 0) { g_aim = FPAim(); g_aim.valid = true; g_aim.kind = 30; g_aim.idx = best; g_aim.prompt = L"[E/右クリック] 次の電車を確認する"; }
			}
		}

		hurtCool = std::max(0.0f, hurtCool - dt); hurtFlash = std::max(0.0f, hurtFlash - dt * 1.6f);
		regenWait = std::max(0.0f, regenWait - dt);
		if (!g_dead && hp < 1000 && regenWait <= 0.0f) {
			hpAcc += 8.0f * dt;
			const int add = (int)hpAcc;
			if (add > 0) { hp = std::min(1000, hp + add); hpAcc -= (float)add; }
		}
		if (g_cam == 5 && g_fpHitKmh >= 0.0f && !g_dead && hurtCool <= 0.0f) {
			const int dmg = hitDamage(g_fpHitKmh);
			if (dmg > 0) {
				hp = std::max(0, hp - dmg); hurtCool = 1.0f; hurtFlash = 1.0f; regenWait = 4.0f; hpAcc = 0.0f;
				if (hp <= 0) { g_dead = true; g_respHover = false; }
				else { wchar_t tb[64]; swprintf_s(tb, L"電車に接触！ -%d", dmg); setToast(tb, 2.0f); }
			}
		}
		pollShaderLoad();   // バックグラウンドのシェーダー読み込みが終わっていれば回収する（適用はしない）
		// 選ばれたものを、いま使える範囲に丸める（未解禁のものは「オフ」の描画）
		bool effPseudo = g_pseudoRT && pseudoOK();
		int effGfx = (!effPseudo && g_gfx != 0 && gfxOK(g_gfx)) ? g_gfx : 0;
		if (effGfx != 0 && !rtReady) {   // レイトレーシングのレベルを選んだ時だけ、BVH(またはBLAS/TLAS)を作る。ここで短時間止まる
			waitFence(g_frameFence);
			g_alloc->Reset(); g_cl->Reset(g_alloc, nullptr);
			if (initRTScene(vWorld, rtV, world, rtMesh)) { rtReady = true; flushInit(); Verts().swap(vWorld); }
			else {
				flushInit();
				MessageBoxW(nullptr, L"レイトレーシング用データの作成に失敗しました。グラフィックレベルを「オフ」に戻します", L"TrainSim", MB_OK);
				g_gfx = 0; g_pseudoRT = false; effGfx = 0; effPseudo = false;
			}
			prevValid = false;   // 止まっていた間の時間が大きな dt にならないよう、時刻の基準をとり直す
			QueryPerformanceCounter(&t0);
		}
		// 使う PSO の組: HW-RT のレベルだけ HW組（DXIL）、それ以外（疑似RT・SW-RT）は SW組（DXBC）
		const bool useHw = effGfx != 0 && g_hwRT && g_bgHwDone.load();
		const PsoSet& S = useHw ? g_setHW : g_setSW;
		g_psoDepth = S.depth; g_psoScene = S.scene; g_psoGlass = S.glass; g_psoPost = S.post; g_psoDn = S.dn;
		M4 vp = mul(lookAt(eye, at, { 0,1,0 }), perspective(1.0f, (float)g_w / g_h, 0.15f, 3000.0f));
		GfxParam gp = effPseudo ? GFX_PSEUDO : GFX[effGfx];
		if (!g_psoDn) gp.dnRad = 0.0f;   // dn パスがまだの間は、ぼかし・グローを省く（描画は止めない）
		const bool lite = (effGfx == 0) && !effPseudo;
		const bool camJump = !prevValid || g_cam != prevCam || fabsf(eye.x - prevEye.x) + fabsf(eye.y - prevEye.y) + fabsf(eye.z - prevEye.z) > 40.0f;

		{
			const bool inBed = (g_cam == 5 && g_fp.seat == 5 && !g_dead);
			bool skipping = false;
			if (inBed && !g_todFrozen) {
				if (!sleepOn) {
					sleepOn = true; sleepT = 0.0f; sleepStart = g_tod; sleepSpan = nightSpan(g_tod);
					if (sleepSpan > 0.0) setToast(L"就寝中… 朝まで早送りします（Eキーで起きるとその時刻で止まります）", 3.5f);
				}
				if (sleepSpan > 0.0) {
					sleepT += dt;
					g_tod = fmod(sleepStart + sleepSpan * (double)std::min(1.0f, sleepT / 5.0f), 24.0);
					skipping = true;
				}
			}
			else sleepOn = false;
			if (!skipping && !g_todFrozen) g_tod = fmod(g_tod + (double)dt * TOD_RATE, 24.0);
		}
		updateSky(g_tod);
		const float nightF = g_nightF;
		const float lampOnF = std::min(1.0f, std::max(0.0f, (nightF - 0.30f) / 0.45f));
		float sky[4] = { 0.52f + (0.01f - 0.52f) * nightF, 0.72f + (0.015f - 0.72f) * nightF, 0.96f + (0.045f - 0.96f) * nightF, 1 };

		{
			g_env[0] = nightF; g_env[1] = lampOnF; g_env[2] = g_env[3] = 0.0f;
			// 前照灯: 回送・車庫内を除き、視点に近い順に最大 HL_TRAIN_MAX 編成の先頭車から出す
			struct HlCand { float dd; size_t ti; };
			std::vector<HlCand> hlc;
			for (size_t ti = 0; ti < NTR; ti++) {
				if (!nearTr[ti] || g_rs.tr[ti].st == TS_YARD) continue;
				const M4& cm = trCW[ti * NUM_CARS];
				const float dd = (cm.m[3][0] - eye.x) * (cm.m[3][0] - eye.x) + (cm.m[3][2] - eye.z) * (cm.m[3][2] - eye.z);
				hlc.push_back({ dd, ti });
			}
			std::sort(hlc.begin(), hlc.end(), [](const HlCand& a, const HlCand& b) { return a.dd < b.dd; });
			for (int h = 0; h < HL_TRAIN_MAX; h++) {
				if (h >= (int)hlc.size()) { g_spd[h][3] = 0.0f; continue; }   // 未使用の枠は無効
				const M4 hm = trCW[hlc[h].ti * NUM_CARS];
				V3 lfw = { hm.m[2][0], hm.m[2][1], hm.m[2][2] }, lc = { hm.m[3][0], hm.m[3][1], hm.m[3][2] };
				V3 lamp = lc + V3{ 0, 1.44f, 0 } + lfw * 10.3f, lrt = rightOf(lfw);
				V3 pL = lamp - lrt * 0.80f, pR = lamp + lrt * 0.80f;
				V3 ld = norm(lfw + V3{ 0, -0.06f, 0 });
				g_sp0[h][0] = pL.x; g_sp0[h][1] = pL.y; g_sp0[h][2] = pL.z; g_sp0[h][3] = 1;
				g_sp1[h][0] = pR.x; g_sp1[h][1] = pR.y; g_sp1[h][2] = pR.z; g_sp1[h][3] = 1;
				g_spd[h][0] = ld.x; g_spd[h][1] = ld.y; g_spd[h][2] = ld.z; g_spd[h][3] = 1;
			}
			g_spp[0] = 0.9925f; g_spp[1] = 0.9135f; g_spp[2] = 220.0f; g_spp[3] = 1.5f;
		}


		M4 rtCW[4]; uint32_t rtType[4];
		{
			struct CC { float d; M4 m; uint32_t type; }; std::vector<CC> cands;
			for (size_t ti = 0; ti < NTR; ti++) {
				if (!nearTr[ti]) continue; for (int i = 0; i < NUM_CARS; i++) {
					const M4& cm = trCW[ti * NUM_CARS + i];
					const float dd = (cm.m[3][0] - eye.x) * (cm.m[3][0] - eye.x) + (cm.m[3][1] - eye.y) * (cm.m[3][1] - eye.y) + (cm.m[3][2] - eye.z) * (cm.m[3][2] - eye.z);
					if (dd < 400.0f * 400.0f) cands.push_back({ dd, cm, (i == 0) ? 1u : (i == NUM_CARS - 1) ? 3u : 2u });
				}
			}
			std::sort(cands.begin(), cands.end(), [](const CC& a, const CC& b) { return a.d < b.d; });
			for (int k = 0; k < 4; k++) {
				if (k < (int)cands.size()) { rtCW[k] = cands[k].m; rtType[k] = cands[k].type; }
				else { rtCW[k] = basisMat({ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 0, -8000.0f, 0 }); rtType[k] = 2; }
			}
		}


		FrameCB cb{};
		cb.vp = vp; cb.invVp = inverse4(vp); cb.prevVp = camJump ? vp : prevVp;
		cb.camPos[0] = eye.x; cb.camPos[1] = eye.y; cb.camPos[2] = eye.z;
		cb.sunDir[0] = g_sunDir.x; cb.sunDir[1] = g_sunDir.y; cb.sunDir[2] = g_sunDir.z; cb.sunDir[3] = nightF;
		memcpy(cb.env, g_env, sizeof(g_env)); memcpy(cb.sp0, g_sp0, sizeof(g_sp0)); memcpy(cb.sp1, g_sp1, sizeof(g_sp1));
		memcpy(cb.spd, g_spd, sizeof(g_spd)); memcpy(cb.spp, g_spp, sizeof(g_spp));
		bool inLitRoom = false;
		if (g_hs.valid && g_cam == 5) {
			const V3 q = hzL(eye);
			if (fabsf(q.x) < 4.3f && fabsf(q.z) < 3.8f) {

				if (q.y > -0.3f && q.y < 2.8f && g_hs.roomLvl[0] > 0.5f) inLitRoom = true;
				if (q.y > 2.4f && q.y < 5.15f && (g_hs.roomLvl[1] > 0.5f || g_hs.lampLvl > 0.5f)) inLitRoom = true;
			}
		}
		if (lampOnF > 0.02f && g_cam == 5 && !g_dead && g_fp.car < 0 && !inLitRoom) {
			const V3 fd = norm(at - eye), fr = norm(cross(V3{ 0, 1, 0 }, fd));
			const V3 fpos = eye + fr * 0.18f + V3{ 0, -0.14f, 0 } + fd * 0.25f;
			cb.flA[0] = fpos.x; cb.flA[1] = fpos.y; cb.flA[2] = fpos.z; cb.flA[3] = 1.0f;
			cb.flB[0] = fd.x; cb.flB[1] = fd.y; cb.flB[2] = fd.z; cb.flB[3] = 0.86f;
			cb.flC[0] = 0.97f; cb.flC[1] = 28.0f; cb.flC[2] = 1.6f * lampOnF;
		}
		cb.rtA[0] = gp.shRays; cb.rtA[1] = gp.sunR; cb.rtA[2] = gp.aoRays; cb.rtA[3] = gp.aoDist;
		cb.rtB[0] = gp.giRays; cb.rtB[1] = gp.refl; cb.rtB[2] = gp.maxDist; cb.rtB[3] = gp.flare;
		cb.rtC[0] = gp.blur; cb.rtC[1] = 1.0f / g_w; cb.rtC[2] = 1.0f / g_h; cb.rtC[3] = (float)g_w / g_h;
		cb.rtE[0] = gp.dnRad; cb.rtE[1] = std::max(gp.dnSig, 0.5f);
		cb.rtD[0] = gp.cell; cb.rtD[1] = gp.lamp; cb.rtD[2] = 0.5f; cb.rtD[3] = 0.035f;
		cb.rtF[0] = (float)fmod(timeAcc, 7200.0); cb.rtF[1] = gp.ssao; cb.rtF[2] = 1.1f; cb.rtF[3] = gp.bloom;
		cb.rtG[0] = gp.shafts; cb.rtG[1] = gp.cloud; cb.rtG[2] = gp.detail; cb.rtG[3] = gp.water;
		cb.rtH[0] = gp.vol; cb.rtH[1] = gp.bounce2;
		cb.rtH[2] = effPseudo ? 1.0f : 0.0f; cb.rtH[3] = 0.0f;
		for (int k = 0; k < 4; k++) { cb.roots[k] = g_rtRoots[k]; cb.triBase[k] = g_rtTriBase[k]; }
		houseFillCB(cb, (float)fmod(timeAcc, 7200.0));
		{
			// 光源の選別: 前照灯の照射範囲に入っている光源を優先し、残りは視点から近い順に詰める。
			// 上限は、ハードウェアRT端末 16 個 / ソフトウェアRT端末 8 個。
			const int lampMax = g_hwRT ? LAMP_HW_MAX : LAMP_SW_MAX;
			struct LampCand { int tier; float d2; int li; };
			std::vector<LampCand> cands;
			cands.reserve(g_lampPos.size());
			for (int li = 0; li < (int)g_lampPos.size(); li++) {
				const V3 d = g_lampPos[li] - eye;
				const float d2 = dot(d, d);
				if (d2 > 60.0f * 60.0f) continue;
				bool inBeam = false;
				for (int h = 0; h < HL_TRAIN_MAX && !inBeam; h++) {
					if (g_spd[h][3] < 0.5f) continue;
					const V3 hd = { g_spd[h][0], g_spd[h][1], g_spd[h][2] };
					for (int side = 0; side < 2 && !inBeam; side++) {
						const float* sp = side ? g_sp1[h] : g_sp0[h];
						const V3 v = g_lampPos[li] - V3{ sp[0], sp[1], sp[2] };
						const float vl = sqrtf(dot(v, v));
						inBeam = vl > 0.001f && vl < g_spp[2] && dot(v, hd) / vl > g_spp[1];
					}
				}
				cands.push_back({ inBeam ? 0 : 1, d2, li });
			}
			std::sort(cands.begin(), cands.end(), [](const LampCand& a, const LampCand& b) {
				return a.tier != b.tier ? a.tier < b.tier : a.d2 < b.d2; });
			const int nPick = std::min(lampMax, (int)cands.size());
			for (int s = 0; s < nPick; s++) {
				const V3& lp = g_lampPos[cands[s].li];
				cb.lampP[s][0] = lp.x; cb.lampP[s][1] = lp.y; cb.lampP[s][2] = lp.z; cb.lampP[s][3] = 1.0f;
			}
			cb.lampI[0] = 0.10f + (1.7f - 0.10f) * lampOnF;
		}
		for (int i = 0; i < 4; i++) {
			cb.carType[i] = rtType[i];
			cb.carW[i] = rtCW[i]; cb.carInv[i] = inverse4(rtCW[i]);
		}

		HudInfo hi2;
		hi2.kmh = focusKmh; hi2.handle = 0; hi2.d = 0.0f; hi2.nextStationName = g_stations[0].name;
		hi2.doorPosLeft = focusDL; hi2.doorPosRight = focusDR; hi2.doorTargetLeft = false; hi2.doorTargetRight = false;
		hi2.cabView = (g_cam == 5 && g_fp.seat == 2); hi2.driving = false; hi2.cam = camNames[g_cam]; hi2.toast = toast; hi2.toastT = toastT;
		hi2.pause = g_pause;
		hi2.rev = 1; hi2.tod = (float)g_tod; hi2.todFrozen = g_todFrozen; hi2.menuSel = g_menuSel; hi2.mouseMode = g_mouseMode;
		hi2.gfx = g_gfx; hi2.pseudo = g_pseudoRT; hi2.pseudoUnlocked = pseudoOK(); hi2.rtUnlocked = rtUnlocked(); hi2.hwRT = g_hwRT; hi2.swGpu = g_swGpu; hi2.gpu = g_gpuName; hi2.rtNote = g_rtNote; hi2.shaderLoading = shaderLoading(); hi2.shaderFailed = g_heavyFailed;
		hi2.hp = hp; hi2.dead = g_dead; hi2.hurt = hurtFlash;
		hi2.ato = 0;
		hi2.cross = (g_cam == 5 && !g_dead);
		hi2.crossHot = (g_cam == 5 && !g_fp.seat && g_aim.valid);
		if (g_cam == 5 && g_fp.seat) hi2.prompt = (g_fp.seat == 5) ? L"[E/右クリック] 起き上がる" : (g_fp.seat == 2 ? L"[E/右クリック] 立ち上がる    1:追従視点" : L"[E/右クリック] 立ち上がる");
		else if (g_cam == 5 && g_aim.valid) hi2.prompt = g_aim.prompt;
		else if (g_cam != 5 && g_driverSeat) hi2.prompt = L"[E/右クリック] 席を立つ    2:一人称視点";
		hi2.info = infoLines; hi2.infoT = infoT;
		if (focusTr >= 0 && g_rs.tr[focusTr].onboard && infoT <= 0.0f) {
			const Train& RT = g_rs.tr[focusTr]; wchar_t ib[128];
			hi2.info.clear();
			if (RT.st == TS_RUN || RT.st == TS_DWELL) { swprintf_s(ib, L"乗車中: %s (%s)  次は%sです  %.0f km/h", TYPE_NAME[RT.type], RT.line == 0 ? L"下り" : L"上り", g_stations[g_order[RT.line][std::min(RT.pos, g_nStops - 1)]].name.c_str(), RT.v * 3.6f); }
			else swprintf_s(ib, L"乗車中: %s  停車中", TYPE_NAME[RT.type]);
			hi2.info.push_back(ib); hi2.infoT = 1.0f;
		}

		buildHud(hi2);
		hudRasterText();


		waitFence(g_frameFence);
		g_alloc->Reset(); g_cl->Reset(g_alloc, nullptr);
		memcpy(g_cbPtr, &cb, sizeof(cb));
		const UINT nhv = (UINT)std::min<size_t>(g_hv.size(), HV_MAX);
		if (nhv) memcpy(hvbPtr, g_hv.data(), nhv * sizeof(HV));
		recordTextCopy();
		ID3D12DescriptorHeap* heaps[] = { g_srvHeap };
		g_cl->SetDescriptorHeaps(1, heaps);
		if (useHw && rtReady) updateTLAS(rtCW, rtType, 4);

		const D3D12_VIEWPORT vpt = { 0, 0, (float)g_w, (float)g_h, 0, 1 };
		const D3D12_RECT sci = { 0, 0, g_w, g_h };
		const D3D12_GPU_VIRTUAL_ADDRESS cbAddr = g_cbUp->GetGPUVirtualAddress();
		const D3D12_CPU_DESCRIPTOR_HANDLE rtvMsArr[2] = { rtvAt(BB_COUNT), rtvAt(BB_COUNT + 1) }, rtvMs = rtvMsArr[0], dsvH = g_dsvHeap->GetCPUDescriptorHandleForHeapStart();

		g_cl->SetGraphicsRootSignature(g_rsScene);
		g_cl->SetGraphicsRootConstantBufferView(1, cbAddr);
		g_cl->SetGraphicsRootDescriptorTable(2, srvGpu(0));
		g_cl->RSSetViewports(1, &vpt); g_cl->RSSetScissorRects(1, &sci);
		g_cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
		g_cl->OMSetRenderTargets(lite ? 1 : 2, rtvMsArr, FALSE, &dsvH);
		g_cl->ClearRenderTargetView(rtvMs, sky, 0, nullptr);
		if (!lite) { const float z4[4] = { 0, 0, 0, 0 }; g_cl->ClearRenderTargetView(rtvMsArr[1], z4, 0, nullptr); }
		g_cl->ClearDepthStencilView(dsvH, D3D12_CLEAR_FLAG_DEPTH, 1.0f, 0, 0, nullptr);


		g_curTrain = false; g_curGlass = false;
		if (!lite) {
			g_cl->SetPipelineState(g_psoDepth);
			drawMesh(track, vp, ident());
			drawMesh(scenery, vp, ident());
		}
		g_cl->SetPipelineState(lite ? g_psoSceneL : g_psoScene);
		drawMesh(track, vp, ident());
		drawMesh(scenery, vp, ident());
		houseDrawOpaque(vp, (float)fmod(timeAcc, 7200.0));
		drawMesh(bldInter, vp, ident());
		{
			buildLampVerts(lampV, eye, timeAcc); dynSet(g_dynLamp, lampV);
			if (g_dynLamp.m.n) drawMesh(g_dynLamp.m, vp, ident());
			for (size_t xi = 0; xi < g_xingObjs.size() && xi < g_rs.cx.size(); xi++) {
				const TFrame& XF = g_xingObjs[xi].F;
				if ((XF.p.x - eye.x) * (XF.p.x - eye.x) + (XF.p.z - eye.z) * (XF.p.z - eye.z) > 500.0f * 500.0f) continue;
				for (int sg = -1; sg <= 1; sg += 2) for (int sz = -1; sz <= 1; sz += 2) drawMesh(g_armMesh, vp, armMatrix(XF, sg, sz, g_rs.cx[xi].arm));
			}
			for (size_t bi = 0; bi < g_boardObjs.size(); bi++) {
				const V3 dd = g_boardObjs[bi].c - eye;
				if (dd.x * dd.x + dd.y * dd.y + dd.z * dd.z > 130.0f * 130.0f) continue;
				ledUpdate((int)bi);
				if (g_led[bi].dm.m.n) drawMesh(g_led[bi].dm.m, vp, ident());
			}
		}

		g_curTrain = true;
		auto smooth01 = [](float t) { t = std::min(1.0f, std::max(0.0f, t)); return t * t * (3.0f - 2.0f * t); };
		const float NEAR_D = 220.0f;
		const M4 cabRot = basisMat({ -1, 0, 0 }, { 0, 1, 0 }, { 0, 0, -1 }, { 0, 0, 0 });
		for (size_t ti = 0; ti < NTR; ti++) {
			if (!nearTr[ti]) continue;
			const Train& T = g_rs.tr[ti];
			for (int i = 0; i < NUM_CARS; i++) {
				const M4& w = trCW[ti * NUM_CARS + i];
				const float ddx = w.m[3][0] - eye.x, ddy = w.m[3][1] - eye.y, ddz = w.m[3][2] - eye.z;
				const float dd = sqrtf(ddx * ddx + ddy * ddy + ddz * ddz);
				if (dd > 1300.0f) continue;
				if (dd > NEAR_D) { drawMesh(rtMesh[(i == 0) ? 0 : (i == NUM_CARS - 1) ? 2 : 1], vp, w); continue; }
				Mesh bodyMesh = (i == 0) ? bodyHead : (i == NUM_CARS - 1) ? bodyTail : bodyMid;
				drawMesh(bodyMesh, vp, w);
				for (int side = 0; side < 2; side++) {
					float pos = (side == 0) ? T.doorL : T.doorR;
					float sgn = (side == 0) ? -1.0f : 1.0f;
					const Mesh& leaf = (side == 0) ? doorLeafL : doorLeafR;
					float plug = smooth01(pos / 0.12f) * LEAF_PLUG;
					float slide = smooth01((pos - 0.08f) / 0.92f) * LEAF_SLIDE;
					for (int k = 0; k < 3; k++) for (int dir = -1; dir <= 1; dir += 2) {
						V3 off = { -sgn * plug, 0.0f, DOOR_Z[k] + dir * (LEAF_OFS + slide) };
						drawMesh(leaf, vp, mul(basisMat({ 1,0,0 }, { 0,1,0 }, { 0,0,1 }, off), w));
					}
				}
				if (i == 0 || i == NUM_CARS - 1) {
					const int en = (i == 0) ? 0 : 1; const float zs = (i == 0) ? 1.0f : -1.0f;
					for (int s = 0; s < 2; s++) drawMesh(cabDoorM[s][en], vp, mul(cabDoorMat(s ? 1.0f : -1.0f, zs, T.cabPos[en * 2 + s]), w));
					if (dd < 70.0f) {
						const M4 cw = (i == 0) ? w : mul(cabRot, w);
						drawMesh(mcHandle, vp, mul(basisMat({ 1,0,0 }, { 0,1,0 }, { 0,0,1 }, { MC_PIVOT[0], MC_PIVOT[1], MC_PIVOT[2] }), cw));
						const float kmhNeedle = std::min(T.v * 3.6f, 120.0f);
						const float p1 = (135.0f - 270.0f * kmhNeedle / 120.0f) * PI / 180.0f;
						drawMesh(needle, vp, mul(basisMat({ cosf(p1), sinf(p1), 0 }, { -sinf(p1), cosf(p1), 0 }, { 0,0,1 }, { DIAL1[0], DIAL1[1], DIAL1[2] }), cw));
						const float p2 = (135.0f - 270.0f * 0.0f) * PI / 180.0f;
						drawMesh(needle, vp, mul(basisMat({ cosf(p2), sinf(p2), 0 }, { -sinf(p2), cosf(p2), 0 }, { 0,0,1 }, { DIAL2[0], DIAL2[1], DIAL2[2] }), cw));
						drawMesh((T.doorL <= 0.0f && T.doorR <= 0.0f) ? lampOn : lampOff, vp, cw);
					}
				}
			}
		}


		g_cl->SetPipelineState(lite ? g_psoGlassL : g_psoGlass);
		g_curGlass = true;

		{
			float bf[4] = { 0.32f, 0.32f, 0.32f, 0.32f };
			setGlassBlend(bf[0]);

			{
				g_curTrain = false; setGlassBlend(0.30f);
				drawMesh(bldGlass, vp, ident());
				g_curTrain = true; setGlassBlend(bf[0]);
			}
			for (size_t ti = 0; ti < NTR; ti++) {
				if (!nearTr[ti]) continue;
				const Train& T = g_rs.tr[ti];
				for (int i = 0; i < NUM_CARS; i++) {
					const M4& w = trCW[ti * NUM_CARS + i];
					const float ddx = w.m[3][0] - eye.x, ddy = w.m[3][1] - eye.y, ddz = w.m[3][2] - eye.z;
					if (ddx * ddx + ddy * ddy + ddz * ddz > NEAR_D * NEAR_D) continue;
					const Mesh& gm = (i == 0) ? glassHead : (i == NUM_CARS - 1) ? glassTail : glassMid;
					drawMesh(gm, vp, w);
					if (i == 0 || i == NUM_CARS - 1) {
						const int en = (i == 0) ? 0 : 1; const float zs = (i == 0) ? 1.0f : -1.0f;
						for (int s = 0; s < 2; s++) drawMesh(cabGlassM[s][en], vp, mul(cabDoorMat(s ? 1.0f : -1.0f, zs, T.cabPos[en * 2 + s]), w));
					}
					for (int side = 0; side < 2; side++) {
						float pos = (side == 0) ? T.doorL : T.doorR;
						float sgn = (side == 0) ? -1.0f : 1.0f;
						const Mesh& dg = (side == 0) ? doorGlassL : doorGlassR;
						float plug = smooth01(pos / 0.12f) * LEAF_PLUG;
						float slide = smooth01((pos - 0.08f) / 0.92f) * LEAF_SLIDE;
						for (int k = 0; k < 3; k++) for (int dir = -1; dir <= 1; dir += 2) {
							V3 off = { -sgn * plug, 0.0f, DOOR_Z[k] + dir * (LEAF_OFS + slide) };
							drawMesh(dg, vp, mul(basisMat({ 1,0,0 }, { 0,1,0 }, { 0,0,1 }, off), w));
						}
					}
				}
			}

			float bf2[4] = { 0.26f, 0.26f, 0.26f, 0.26f };
			setGlassBlend(bf2[0]);
			for (size_t ti = 0; ti < NTR; ti++) {
				if (!nearTr[ti]) continue;
				for (int i = 0; i < NUM_CARS; i += NUM_CARS - 1) {
					const M4& w = trCW[ti * NUM_CARS + i];
					const float ddx = w.m[3][0] - eye.x, ddy = w.m[3][1] - eye.y, ddz = w.m[3][2] - eye.z;
					if (ddx * ddx + ddy * ddy + ddz * ddz > NEAR_D * NEAR_D) continue;
					drawMesh(i == 0 ? windF : windR, vp, w);
				}
			}
			houseDrawGlass(vp, (float)fmod(timeAcc, 7200.0), { 1, 0, 0 });


		}

		g_curGlass = false; g_curTrain = false;


		const UINT bi = g_sc->GetCurrentBackBufferIndex();
		const bool dn = gp.dnRad >= 1.0f;
		if (lite) {
			D3D12_RESOURCE_BARRIER b[2] = {
				trans(g_msColor, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_RESOLVE_SOURCE),
				trans(g_bb[bi], D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RESOLVE_DEST) };
			g_cl->ResourceBarrier(2, b);
			g_cl->ResolveSubresource(g_bb[bi], 0, g_msColor, 0, DXGI_FORMAT_R8G8B8A8_UNORM);
			D3D12_RESOURCE_BARRIER c[2] = {
				trans(g_msColor, D3D12_RESOURCE_STATE_RESOLVE_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET),
				trans(g_bb[bi], D3D12_RESOURCE_STATE_RESOLVE_DEST, D3D12_RESOURCE_STATE_RENDER_TARGET) };
			g_cl->ResourceBarrier(2, c);
		}
		else {
			D3D12_RESOURCE_BARRIER b[5] = {
				trans(g_msColor, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_RESOLVE_SOURCE),
				trans(g_resolved, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RESOLVE_DEST),
				trans(g_msAlb, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_RESOLVE_SOURCE),
				trans(g_resAlb, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RESOLVE_DEST),
				trans(g_msDepth, D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE) };
			g_cl->ResourceBarrier(5, b);
			g_cl->ResolveSubresource(g_resolved, 0, g_msColor, 0, DXGI_FORMAT_R8G8B8A8_UNORM);
			g_cl->ResolveSubresource(g_resAlb, 0, g_msAlb, 0, DXGI_FORMAT_R8G8B8A8_UNORM);
			D3D12_RESOURCE_BARRIER c[5] = {
				trans(g_resolved, D3D12_RESOURCE_STATE_RESOLVE_DEST, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE),
				trans(g_msColor, D3D12_RESOURCE_STATE_RESOLVE_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET),
				trans(g_resAlb, D3D12_RESOURCE_STATE_RESOLVE_DEST, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE),
				trans(g_msAlb, D3D12_RESOURCE_STATE_RESOLVE_SOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET),
				trans(g_bb[bi], D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET) };
			g_cl->ResourceBarrier(5, c);
		}
		g_cl->SetGraphicsRootSignature(g_rsPost);
		g_cl->SetGraphicsRootConstantBufferView(1, cbAddr);
		g_cl->RSSetViewports(1, &vpt); g_cl->RSSetScissorRects(1, &sci);
		g_cl->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
		if (dn) {
			D3D12_RESOURCE_BARRIER e0 = trans(g_dn, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_RENDER_TARGET);
			g_cl->ResourceBarrier(1, &e0);
			const D3D12_CPU_DESCRIPTOR_HANDLE rtvDn = rtvAt(BB_COUNT + 2);
			g_cl->OMSetRenderTargets(1, &rtvDn, FALSE, nullptr);
			g_cl->SetGraphicsRootDescriptorTable(2, srvGpu(3));
			g_cl->SetPipelineState(g_psoDn);
			g_cl->DrawInstanced(3, 1, 0, 0);
			D3D12_RESOURCE_BARRIER e1 = trans(g_dn, D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);
			g_cl->ResourceBarrier(1, &e1);
		}
		const D3D12_CPU_DESCRIPTOR_HANDLE rtvBB = rtvAt(bi);
		g_cl->OMSetRenderTargets(1, &rtvBB, FALSE, nullptr);
		g_cl->SetGraphicsRootDescriptorTable(2, srvGpu(dn ? 7 : 3));
		if (!lite) {
			g_cl->SetPipelineState(g_psoPost);
			g_cl->DrawInstanced(3, 1, 0, 0);
		}

		{
			const float scr[4] = { (float)g_w, (float)g_h, 0, 0 };
			g_cl->SetGraphicsRoot32BitConstants(0, 4, scr, 0);
			g_cl->SetPipelineState(g_psoHud);
			D3D12_VERTEX_BUFFER_VIEW hv{ hvbUp->GetGPUVirtualAddress(), (UINT)(nhv * sizeof(HV)), (UINT)sizeof(HV) };
			g_cl->IASetVertexBuffers(0, 1, &hv);
			if (nhv) g_cl->DrawInstanced(nhv, 1, 0, 0);
			g_cl->SetPipelineState(g_psoOv);
			g_cl->DrawInstanced(3, 1, 0, 0);
		}
		{
			D3D12_RESOURCE_BARRIER d[2] = {
				trans(g_bb[bi], D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT),
				trans(g_msDepth, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE, D3D12_RESOURCE_STATE_DEPTH_WRITE) };
			g_cl->ResourceBarrier(lite ? 1 : 2, d);
		}
		g_cl->Close();
		ID3D12CommandList* lists[] = { g_cl };
		g_q->ExecuteCommandLists(1, lists);
		g_sc->Present(1, 0);
		g_fenceVal++; g_q->Signal(g_fence, g_fenceVal); g_frameFence = g_fenceVal;

		prevVp = vp; prevEye = eye; prevCam = g_cam; prevValid = true;

		fpsAcc += dt; fpsFrames++;
		if (fpsAcc >= 0.5f) {
			wchar_t title[256];
			swprintf_s(title, L"SilkHat | %.0f FPS | WASD:移動 Shift:ダッシュ Space:ジャンプ E:座る/立つ/操作(赤い標識で次の電車) Esc:メニュー | グラフィック:%s",
				fpsFrames / fpsAcc, GFX_NAME[g_gfx]);
			SetWindowTextW(hwnd, title);
			fpsAcc = 0; fpsFrames = 0;
		}
	}
	if (mouseCaptured) { ClipCursor(nullptr); while (ShowCursor(TRUE) < 0) {} }
	shutdownShaderLoad();
	gpuIdle();
	return 0;
}