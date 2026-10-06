/* SA Destruction v2.0 - Open World Surface Survey, GTA SA US 1.0 x86
 * This ASI does not write game memory and uses NO modloader assets.
 * F8 = native camera ray; inspect hit world-model render, collision, instance.
 * F7 = clear marker; NUM7/NUM9 intentionally disabled until mesh is mapped.
 * Native GTA calls are still made from a worker thread; experiment only.
 */
typedef void* HANDLE;
typedef void* HMODULE;
typedef void* HWND;
typedef void* LPVOID;
typedef const void* LPCVOID;
typedef const char* LPCSTR;
typedef char* LPSTR;
typedef unsigned long DWORD;
typedef unsigned long SIZE_T;
typedef long BOOL;
typedef short SHORT;
typedef unsigned short WORD;
typedef unsigned char BYTE;
typedef unsigned long* LPDWORD;
#define WINAPI __stdcall
#define DLLIMPORT __declspec(dllimport)
#define INVALID_HANDLE_VALUE ((HANDLE)-1)
#define GENERIC_WRITE 0x40000000UL
#define FILE_SHARE_READ 1UL
#define OPEN_ALWAYS 4UL
#define FILE_ATTRIBUTE_NORMAL 0x80UL
#define FILE_END 2UL
#define DLL_PROCESS_ATTACH 1
#define DLL_PROCESS_DETACH 0
#define MAX_PATH 260
#define VK_F8 0x77
#define VK_F7 0x76
#define VK_NUMPAD7 0x67
#define VK_NUMPAD9 0x69
DLLIMPORT DWORD WINAPI GetModuleFileNameA(HMODULE,LPSTR,DWORD);
DLLIMPORT BOOL WINAPI DisableThreadLibraryCalls(HMODULE);
DLLIMPORT HANDLE WINAPI CreateThread(LPVOID,DWORD,DWORD (WINAPI*)(LPVOID),LPVOID,DWORD,LPDWORD);
DLLIMPORT BOOL WINAPI CloseHandle(HANDLE);
DLLIMPORT HANDLE WINAPI CreateFileA(LPCSTR,DWORD,DWORD,LPVOID,DWORD,DWORD,HANDLE);
DLLIMPORT BOOL WINAPI WriteFile(HANDLE,LPCVOID,DWORD,LPDWORD,LPVOID);
DLLIMPORT DWORD WINAPI SetFilePointer(HANDLE,long,long*,DWORD);
DLLIMPORT void WINAPI Sleep(DWORD);
DLLIMPORT DWORD WINAPI GetCurrentProcessId(void);
DLLIMPORT HANDLE WINAPI GetCurrentProcess(void);
DLLIMPORT BOOL WINAPI ReadProcessMemory(HANDLE,LPCVOID,LPVOID,SIZE_T,SIZE_T*);
DLLIMPORT DWORD WINAPI GetTickCount(void);
DLLIMPORT HWND WINAPI GetForegroundWindow(void);
DLLIMPORT DWORD WINAPI GetWindowThreadProcessId(HWND,LPDWORD);
DLLIMPORT SHORT WINAPI GetAsyncKeyState(int);
int _fltused=0;
typedef struct {float x,y,z;} Vec3;
static HMODULE g_self;
static volatile long g_shutdown;
static char g_logpath[MAX_PATH];
static int g_marker_valid;
static DWORD g_marker_started;
static Vec3 g_marker_pos,g_marker_normal;
static DWORD slen(const char*s){DWORD n=0;while(s[n]&&n<950)n++;return n;}
static void logline(const char*s){
 HANDLE h=CreateFileA(g_logpath,GENERIC_WRITE,FILE_SHARE_READ,0,OPEN_ALWAYS,FILE_ATTRIBUTE_NORMAL,0);
 if(h==INVALID_HANDLE_VALUE)return;
 SetFilePointer(h,0,0,FILE_END);DWORD n=0;
 WriteFile(h,s,slen(s),&n,0);WriteFile(h,"\r\n",2,&n,0);CloseHandle(h);
}
static int cat(char*s,int p,int cap,const char*t){while(*t&&p<cap-1)s[p++]=*t++;s[p]=0;return p;}
static int dec(char*s,int p,int cap,DWORD n){char b[12];int j=0;do{b[j++]=(char)('0'+n%10);n/=10;}while(n&&j<11);while(j&&p<cap-1)s[p++]=b[--j];s[p]=0;return p;}
static int hex(char*s,int p,int cap,DWORD n){const char*x="0123456789ABCDEF";p=cat(s,p,cap,"0x");for(int j=7;j>=0;j--)if(p<cap-1)s[p++]=x[(n>>(4*j))&15];s[p]=0;return p;}
static DWORD u32(const BYTE*b){return (DWORD)b[0]|((DWORD)b[1]<<8)|((DWORD)b[2]<<16)|((DWORD)b[3]<<24);}
static DWORD u16(const BYTE*b){return (DWORD)b[0]|((DWORD)b[1]<<8);}
static float f32(const BYTE*b){union {DWORD d;float f;}u;u.d=u32(b);return u.f;}
static float ab(float x){return x<0.f?-x:x;}
static int number_f(char*s,int p,int cap,float f){
 if(f!=f||ab(f)>1000000.f)return cat(s,p,cap,"INVALID");
 if(f<0){p=cat(s,p,cap,"-");f=-f;}
 DWORD a=(DWORD)f,frac=(DWORD)((f-(float)a)*1000.f+0.5f);
 if(frac>=1000){a++;frac=0;}
 p=dec(s,p,cap,a);p=cat(s,p,cap,".");
 if(frac<100)p=cat(s,p,cap,"0");if(frac<10)p=cat(s,p,cap,"0");return dec(s,p,cap,frac);
}
static int vprint(char*s,int p,int cap,const Vec3*v){p=cat(s,p,cap,"(");p=number_f(s,p,cap,v->x);p=cat(s,p,cap,",");p=number_f(s,p,cap,v->y);p=cat(s,p,cap,",");p=number_f(s,p,cap,v->z);return cat(s,p,cap,")");}
static int readmem(DWORD a,void*out,SIZE_T n){SIZE_T got=0;return a>=0x10000UL&&a<0x7FFF0000UL&&n&&n<=32768&&a<0x7FFF0000UL-(DWORD)n&&ReadProcessMemory(GetCurrentProcess(),(LPCVOID)a,out,n,&got)&&got==n;}
static int position_ok(const Vec3*v){return v->x==v->x&&v->y==v->y&&v->z==v->z&&ab(v->x)<12000.f&&ab(v->y)<12000.f&&ab(v->z)<10000.f;}
static float sqrroot(float a){float x=a>1.f?a:1.f;for(int i=0;i<12;i++)x=0.5f*(x+a/x);return x;}
typedef unsigned char (__cdecl *LineOfSight)(const Vec3*,const Vec3*,void*,void**,BYTE,BYTE,BYTE,BYTE,BYTE,BYTE,BYTE,BYTE);
typedef void* (__cdecl *PlaceMarkerOriented)(DWORD,WORD,Vec3*,float,BYTE,BYTE,BYTE,BYTE,WORD,float,short,float,float,float,int);
static void inspect_hit(DWORD entity,WORD model,const Vec3*p){
 char s[360];int n=0;BYTE e[0x38]={0};
 if(!entity||!readmem(entity,e,sizeof(e))){logline("[v2.0] ENTITY: unreadable CEntity; no further inspection.");return;}
 DWORD rw=u32(e+0x18),transform=u32(e+0x14),lod=u32(e+0x30);
 Vec3 origin={f32(e+4),f32(e+8),f32(e+12)};
 BYTE mat[0x40];
 if(transform && readmem(transform,mat,sizeof(mat))){Vec3 from={f32(mat+0x30),f32(mat+0x34),f32(mat+0x38)};if(position_ok(&from))origin=from;}
 n=cat(s,n,360,"[v2.0] ENTITY model=");n=dec(s,n,360,model);
 n=cat(s,n,360," ptr=");n=hex(s,n,360,entity);
 n=cat(s,n,360," flags=");n=hex(s,n,360,e[0x1C]);
 n=cat(s,n,360," origin=");n=vprint(s,n,360,&origin);
 n=cat(s,n,360," rw=");n=hex(s,n,360,rw);
 n=cat(s,n,360," lodPtr=");n=hex(s,n,360,lod);logline(s);
 if(lod&&lod!=entity){BYTE lodmodel[2]={0};if(readmem(lod+0x22,lodmodel,2)){n=0;s[0]=0;n=cat(s,n,360,"[v2.0] LOD entity model=");n=dec(s,n,360,u16(lodmodel));logline(s);}}
 if(rw){BYTE atom[0x20]={0};if(readmem(rw,atom,sizeof(atom))){
   DWORD type=u32(atom),geom=u32(atom+0x18);n=0;s[0]=0;
   n=cat(s,n,360,"[v2.0] RENDER type=");n=dec(s,n,360,type);n=cat(s,n,360," geometry=");n=hex(s,n,360,geom);logline(s);
   if(type==1&&geom){BYTE b[0x24]={0};if(readmem(geom,b,sizeof(b))){DWORD t=u32(b+0x10),v=u32(b+0x14),mt=u32(b+0x18);n=0;s[0]=0;n=cat(s,n,360,"[v2.0] RENDER_ATOMIC triangles=");n=dec(s,n,360,t);n=cat(s,n,360," vertices=");n=dec(s,n,360,v);n=cat(s,n,360," morphs=");n=dec(s,n,360,mt);logline(s);}}
   else if(type!=1)logline("[v2.0] RENDER not RpAtomic (often Clump/LOD); geometry not yet mapped.");
  }}
 DWORD info=0;
 if(model<20000U && readmem(0xA9B0C8UL+(DWORD)model*4UL,&info,4)&&info){
  DWORD colmodel=0,coldata=0;BYTE b[0x28]={0};
  if(readmem(info+0x14,&colmodel,4)&&colmodel&&readmem(colmodel+0x2C,&coldata,4)&&coldata&&readmem(coldata,b,sizeof(b))){
   DWORD num=u16(b+4),verts=u32(b+0x14),tris=u32(b+0x18),planes=u32(b+0x1C);
   n=0;s[0]=0;n=cat(s,n,360,"[v2.0] COLLISION faces=");n=dec(s,n,360,num);
   n=cat(s,n,360," vertPtr=");n=hex(s,n,360,verts);n=cat(s,n,360," triPtr=");n=hex(s,n,360,tris);
   n=cat(s,n,360," planesPtr=");n=hex(s,n,360,planes);logline(s);
  }else logline("[v2.0] COLLISION: no readable CColData; model may use another collision layout.");
 }else logline("[v2.0] COLLISION: no registered CBaseModelInfo for this entity.");
 (void)p;
}
static void target(){
 BYTE cam[0x48]={0};if(!readmem(0xB6F028UL+0x974UL,cam,sizeof(cam))){logline("[v2.0] F8 ERROR: camera unreadable.");return;}
 Vec3 from={f32(cam+0x30),f32(cam+0x34),f32(cam+0x38)},dir={f32(cam+0x10),f32(cam+0x14),f32(cam+0x18)};
 if(!position_ok(&from)||ab(dir.x)>2||ab(dir.y)>2||ab(dir.z)>2||dir.x!=dir.x||dir.y!=dir.y||dir.z!=dir.z){logline("[v2.0] F8 ERROR: invalid camera transform.");return;}
 float n=sqrroot(dir.x*dir.x+dir.y*dir.y+dir.z*dir.z);if(n<0.25f||n>2.f){logline("[v2.0] F8 ERROR: invalid look direction.");return;}
 Vec3 end={from.x+dir.x*180.f/n,from.y+dir.y*180.f/n,from.z+dir.z*180.f/n};
 BYTE point[0x2C]={0};void*obj=0;
 unsigned char hit=((LineOfSight)0x56BA00UL)(&from,&end,point,&obj,1,0,0,1,0,0,0,0);
 if(!hit){g_marker_valid=0;logline("[v2.0] WORLD MISS: no building/object within 180m.");return;}
 Vec3 p={f32(point),f32(point+4),f32(point+8)};
 if(!position_ok(&p)){logline("[v2.0] WORLD ERROR: hit point invalid.");return;}
 WORD model=0;BYTE m[2];if(obj&&readmem((DWORD)obj+0x22,m,2))model=(WORD)u16(m);
 char s[360];int k=0;s[0]=0;k=cat(s,k,360,"[v2.0] WORLD HIT model=");k=dec(s,k,360,model);
 k=cat(s,k,360," point=");k=vprint(s,k,360,&p);logline(s);
 inspect_hit((DWORD)obj,model,&p);
 Vec3 norm={f32(point+0x10),f32(point+0x14),f32(point+0x18)};
 float len=sqrroot(norm.x*norm.x+norm.y*norm.y+norm.z*norm.z);
 if(!position_ok(&norm)||len<0.5f||len>1.5f){g_marker_valid=0;logline("[v2.0] NORMAL invalid: marker suppressed.");return;}
 norm.x/=len;norm.y/=len;norm.z/=len;
 if(norm.x*dir.x+norm.y*dir.y+norm.z*dir.z>0){norm.x=-norm.x;norm.y=-norm.y;norm.z=-norm.z;}
 g_marker_normal=norm;g_marker_pos=(Vec3){p.x+norm.x*0.08f,p.y+norm.y*0.08f,p.z+norm.z*0.08f};
 g_marker_started=GetTickCount();g_marker_valid=1;
}
static void marker(){
 if(!g_marker_valid)return;
 if((DWORD)(GetTickCount()-g_marker_started)>=8000UL){g_marker_valid=0;return;}
 ((PlaceMarkerOriented)0x725120UL)(0x53415720UL,4,&g_marker_pos,0.8f,255,190,32,165,1024,0.15f,0,g_marker_normal.x,g_marker_normal.y,g_marker_normal.z,0);
}
static DWORD WINAPI worker(LPVOID unused){
 (void)unused;
 DWORD len=GetModuleFileNameA(g_self,g_logpath,MAX_PATH),start=0,dot=len;
 if(!len||len>=MAX_PATH-5)return 1;
 for(DWORD i=0;i<len;i++){if(g_logpath[i]=='\\'||g_logpath[i]=='/')start=i+1;if(g_logpath[i]=='.')dot=i;}
 if(dot<start)dot=len;
 g_logpath[dot++]='.';g_logpath[dot++]='l';g_logpath[dot++]='o';g_logpath[dot++]='g';g_logpath[dot]=0;
 logline("[SA Destruction v2.0] OPEN WORLD SURVEY: F8 world surface scan, F7 clear; Num7/Num9 intentionally DISABLED. READ-ONLY GAME MEMORY.");
 int l8=0,l7=0,l9=0;
 while(!g_shutdown){DWORD pid=0;HWND fg=GetForegroundWindow();if(fg)GetWindowThreadProcessId(fg,&pid);int focus=pid==GetCurrentProcessId();
  int k8=focus&&((GetAsyncKeyState(VK_F8)&(SHORT)0x8000)!=0);
  int k7=focus&&((GetAsyncKeyState(VK_F7)&(SHORT)0x8000)!=0);
  int k9=focus&&((GetAsyncKeyState(VK_NUMPAD9)&(SHORT)0x8000)!=0);
  if(k8&&!l8)target();
  if(k7&&!l7){g_marker_valid=0;logline("[v2.0] Marker cleared.");}
  if(k9&&!l9)logline("[v2.0] NUM9 SAFE BLOCK: terrain write disabled until native world-model geometry/LOD validated.");
  if(focus)marker();l8=k8;l7=k7;l9=k9;Sleep(60);
 }
 return 0;
}
BOOL WINAPI DllMain(HMODULE module,DWORD reason,LPVOID x){(void)x;if(reason==DLL_PROCESS_ATTACH){g_self=module;g_shutdown=0;DisableThreadLibraryCalls(module);HANDLE t=CreateThread(0,0,worker,0,0,0);if(t)CloseHandle(t);}else if(reason==DLL_PROCESS_DETACH)g_shutdown=1;return 1;}
