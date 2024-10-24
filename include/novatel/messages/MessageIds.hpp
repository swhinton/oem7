/**
 * Copyright 2024 Scott Hinton
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the “Software”), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
 * OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

namespace novatel::oem7 {
enum class MessageId : uint16_t {
  PASSAUX = 690,                   // LOG
  FILETRANSFER = 2109,             // COMMAND
  TRANSFERPORTSTATUS = 2114,       // LOG
  LOG = 1,                         // COMMAND
  INTERFACEMODE = 3,               // COMMAND
  COM = 4,                         // COMMAND
  LOGLIST = 5,                     // LOG
  RESET = 18,                      // COMMAND
  SAVECONFIG = 19,                 // COMMAND
  FRESET = 20,                     // COMMAND
  MODEL = 22,                      // COMMAND
  UNLOG = 36,                      // COMMAND
  VERSION = 37,                    // LOG
  UNLOGALL = 38,                   // COMMAND
  AUTH = 49,                       // COMMAND
  PORTSTATS = 72,                  // LOG
  RXSTATUS = 93,                   // LOG
  RXSTATUSEVENT = 94,              // LOG
  STATUSCONFIG = 95,               // COMMAND
  ANTENNAPOWER = 98,               // COMMAND
  RXCONFIG = 128,                  // LOG
  SEND = 177,                      // COMMAND
  SENDHEX = 178,                   // COMMAND
  NVMRESTORE = 197,                // COMMAND
  VALIDMODELS = 206,               // LOG
  PASSCOM1 = 233,                  // LOG
  PASSCOM2 = 234,                  // LOG
  PASSCOM3 = 235,                  // LOG
  COMCONFIG = 317,                 // LOG
  PASSXCOM1 = 405,                 // LOG
  PASSXCOM2 = 406,                 // LOG
  COMCONTROL = 431,                // COMMAND
  SOFTLOADCOMMIT = 475,            // COMMAND
  SOFTLOADRESET = 476,             // COMMAND
  SOFTLOADSREC = 477,              // COMMAND
  PASSUSB1 = 607,                  // LOG
  PASSUSB2 = 608,                  // LOG
  PASSUSB3 = 609,                  // LOG
  PASSXCOM3 = 795,                 // LOG
  NMEATALKER = 861,                // COMMAND
  CANCONFIG = 884,                 // COMMAND
  TUNNELESCAPE = 962,              // COMMAND
  HWMONITOR = 963,                 // LOG
  CONFIGCODE = 1041,               // COMMAND
  CHANCONFIGLIST = 1148,           // LOG
  SELECTCHANCONFIG = 1149,         // COMMAND
  SOFTLOADDATA = 1218,             // COMMAND
  SOFTLOADSETUP = 1219,            // COMMAND
  SOFTLOADSTATUS = 1235,           // LOG
  IPCONFIG = 1243,                 // COMMAND
  DNSCONFIG = 1244,                // COMMAND
  ETHCONFIG = 1245,                // COMMAND
  SERIALCONFIG = 1246,             // COMMAND
  ECHO = 1247,                     // COMMAND
  ICOMCONFIG = 1248,               // COMMAND
  NTRIPCONFIG = 1249,              // COMMAND
  PASSICOM1 = 1250,                // LOG
  PASSICOM2 = 1251,                // LOG
  PASSICOM3 = 1252,                // LOG
  PASSNCOM1 = 1253,                // LOG
  PASSNCOM2 = 1254,                // LOG
  PASSNCOM3 = 1255,                // LOG
  ETHSTATUS = 1288,                // LOG
  IPSTATUS = 1289,                 // LOG
  SOFTLOADFILE = 1302,             // COMMAND
  MODELFEATURES = 1329,            // LOG
  PASSTHROUGH = 1342,              // LOG
  NTRIPSOURCETABLE = 1343,         // COMMAND
  SOURCETABLE = 1344,              // LOG
  PASSCOM4 = 1384,                 // LOG
  PASSICOM4 = 1385,                // LOG
  PROFILE = 1411,                  // COMMAND
  PROFILEINFO = 1412,              // LOG
  SERIALPROTOCOL = 1444,           // COMMAND
  NMEAVERSION = 1574,              // COMMAND
  IPSERVICE = 1575,                // COMMAND
  PASSCOM5 = 1576,                 // LOG
  PASSCOM6 = 1577,                 // LOG
  SETADMINPASSWORD = 1579,         // COMMAND
  IPSTATS = 1669,                  // LOG
  LOGIN = 1671,                    // COMMAND
  LOGOUT = 1672,                   // COMMAND
  SAVEETHERNETDATA = 1679,         // COMMAND
  PASSCOM7 = 1701,                 // LOG
  PASSCOM8 = 1702,                 // LOG
  PASSCOM9 = 1703,                 // LOG
  PASSCOM10 = 1704,                // LOG
  UPTIME = 1777,                   // LOG
  NMEAFORMAT = 1861,               // COMMAND
  RADARSTATUS = 1877,              // LOG
  RADARCONFIG = 1878,              // COMMAND
  PGNCONFIG = 1892,                // COMMAND
  CCOMCONFIG = 1902,               // COMMAND
  J1939CONFIG = 1903,              // COMMAND
  J1939STATUS = 1907,              // LOG
  NVMUSERDATA = 1970,              // COMMAND
  SAFEMODESTATUS = 2060,           // LOG
  FILELIST = 2100,                 // LOG
  FILETRANSFERSTATUS = 2101,       // LOG
  FILESYSTEMSTATUS = 2104,         // LOG
  USBSTICKEJECT = 2115,            // COMMAND
  FILECONFIG = 2116,               // COMMAND
  FILEMEDIACONFIG = 2117,          // COMMAND
  PASSICOM5 = 2119,                // LOG
  PASSICOM6 = 2120,                // LOG
  PASSICOM7 = 2121,                // LOG
  FILESTATUS = 2127,               // LOG
  FILEROTATECONFIG = 2133,         // COMMAND
  FILEAUTOTRANSFER = 2135,         // COMMAND
  LUAFILESYSTEMSTATUS = 2150,      // LOG
  LUAFILELIST = 2151,              // LOG
  LUASTATUS = 2152,                // LOG
  LUA = 2181,                      // COMMAND
  FILEDELETE = 2190,               // COMMAND
  USERI2CREAD = 2232,              // COMMAND
  USERI2CWRITE = 2233,             // COMMAND
  USERI2CRESPONSE = 2234,          // LOG
  LUAOUTPUT = 2240,                // LOG
  PPPSEEDSTORESTATUS = 2251,       // LOG
  NMEABEIDOUTALKER = 2258,         // COMMAND
  ANTENNATYPE = 2281,              // COMMAND
  USERANTENNA = 2282,              // LOG
  TERRASTARAUTOCHANCONFIG = 2284,  // COMMAND
  GEODETICDATUM = 2295,            // COMMAND
  GEODETICDATUMS = 2296,           // LOG
  DATUMTRANSFORMATION = 2297,      // COMMAND
  DATUMTRANSFORMATIONS = 2298,     // LOG
  USERI2CBITRATE = 2383,           // COMMAND
  GPSEPHEM = 7,                    // LOG
  IONUTC = 8,                      // LOG
  CLOCKADJUST = 15,                // COMMAND
  RAWGPSSUBFRAME = 25,             // LOG
  CLOCKSTEERING = 26,              // LOG
  ASSIGN = 27,                     // COMMAND
  ASSIGNALL = 28,                  // COMMAND
  UNASSIGN = 29,                   // COMMAND
  UNASSIGNALL = 30,                // COMMAND
  RAWEPHEM = 41,                   // LOG
  RANGE = 43,                      // LOG
  SATVIS = 48,                     // LOG
  ECUTOFF = 50,                    // COMMAND
  ALMANAC = 73,                    // LOG
  RAWALM = 74,                     // LOG
  TRACKSTAT = 83,                  // LOG
  TIME = 101,                      // LOG
  SETAPPROXTIME = 102,             // COMMAND
  LOCKOUT = 137,                   // COMMAND
  UNLOCKOUT = 138,                 // COMMAND
  UNLOCKOUTALL = 139,              // COMMAND
  RANGECMP = 140,                  // LOG
  EXTERNALCLOCK = 230,             // COMMAND
  MARKTIME = 231,                  // LOG
  FREQUENCYOUT = 232,              // COMMAND
  DYNAMICS = 258,                  // COMMAND
  RAWGPSWORD = 407,                // LOG
  ADJUST1PPS = 429,                // COMMAND
  CLOCKCALIBRATE = 430,            // COMMAND
  TIMESYNC = 492,                  // LOG
  CLOCKOFFSET = 596,               // COMMAND
  PPSCONTROL = 613,                // COMMAND
  MARKCONTROL = 614,               // COMMAND
  MARK2TIME = 616,                 // LOG
  RANGEGPSL1 = 631,                // LOG
  GLOALMANAC = 718,                // LOG
  GLOCLOCK = 719,                  // LOG
  GLORAWALM = 720,                 // LOG
  GLORAWFRAME = 721,               // LOG
  GLORAWSTRING = 722,              // LOG
  GLOEPHEMERIS = 723,              // LOG
  GLOECUTOFF = 735,                // COMMAND
  GLORAWEPHEM = 792,               // LOG
  FORCEGPSL2CODE = 796,            // COMMAND
  CNOUPDATE = 849,                 // COMMAND
  RAWSBASFRAME = 973,              // LOG
  SBAS0 = 976,                     // LOG
  SBAS1 = 977,                     // LOG
  SBAS10 = 978,                    // LOG
  SBAS17 = 980,                    // LOG
  SBAS18 = 981,                    // LOG
  SBAS2 = 982,                     // LOG
  SBAS24 = 983,                    // LOG
  SBAS25 = 984,                    // LOG
  SBAS26 = 985,                    // LOG
  SBAS3 = 987,                     // LOG
  SBAS4 = 992,                     // LOG
  SBAS5 = 994,                     // LOG
  SBAS7 = 996,                     // LOG
  SBAS9 = 997,                     // LOG
  SBASECUTOFF = 1000,              // COMMAND
  DLLTIMECONST = 1011,             // COMMAND
  SATVIS2 = 1043,                  // LOG
  RAWCNAVFRAME = 1066,             // LOG
  MARK3TIME = 1075,                // LOG
  MARK4TIME = 1076,                // LOG
  GALECUTOFF = 1114,               // COMMAND
  GALALMANAC = 1120,               // LOG
  GALCLOCK = 1121,                 // LOG
  GALEPHEMERIS = 1122,             // LOG
  GALFNAVRAWALMANAC = 1123,        // LOG
  GALFNAVRAWEPHEMERIS = 1124,      // LOG
  GALINAVRAWALMANAC = 1125,        // LOG
  GALINAVRAWEPHEMERIS = 1126,      // LOG
  GALIONO = 1127,                  // LOG
  MARK1TIME = 1130,                // LOG
  SETUTCLEAPSECONDS = 1150,        // COMMAND
  LBANDTRACKSTAT = 1201,           // LOG
  FORCEGLOL2CODE = 1217,           // COMMAND
  SETTIMEBASE = 1237,              // COMMAND
  RANGECMP2 = 1273,                // LOG
  GALINAVEPHEMERIS = 1309,         // LOG
  GALFNAVEPHEMERIS = 1310,         // LOG
  TRACKSV = 1326,                  // COMMAND
  QZSSRAWSUBFRAME = 1330,          // LOG
  QZSSRAWEPHEM = 1331,             // LOG
  QZSSEPHEMERIS = 1336,            // LOG
  QZSSRAWALMANAC = 1345,           // LOG
  QZSSALMANAC = 1346,              // LOG
  QZSSIONUTC = 1347,               // LOG
  AUTHCODES = 1348,                // LOG
  QZSSECUTOFF = 1350,              // COMMAND
  RTKTRACKINGCONTROL = 1351,       // COMMAND
  GALFNAVRAWPAGE = 1413,           // LOG
  GALINAVRAWWORD = 1414,           // LOG
  SATXYZ2 = 1451,                  // LOG
  DOPPLERWINDOW = 1512,            // COMMAND
  APPROXPOSTIMEOUT = 1513,         // COMMAND
  QZSSRAWCNAVMESSAGE = 1530,       // LOG
  DATADECODESIGNAL = 1532,         // COMMAND
  BDSECUTOFF = 1582,               // COMMAND
  BDSALMANAC = 1584,               // LOG
  BDSIONO = 1590,                  // LOG
  BDSCLOCK = 1607,                 // LOG
  EVENTOUTCONTROL = 1636,          // COMMAND
  EVENTINCONTROL = 1637,           // COMMAND
  RFINPUTGAIN = 1658,              // COMMAND
  BDSRAWNAVSUBFRAME = 1695,        // LOG
  BDSEPHEMERIS = 1696,             // LOG
  ASSIGNLBANDBEAM = 1733,          // COMMAND
  RANGECMP3 = 1734,                // LOG
  ELEVATIONCUTOFF = 1735,          // COMMAND
  PPSCONTROL2 = 1740,              // COMMAND
  ITSPECTRALANALYSIS = 1967,       // COMMAND
  ITPSDFINAL = 1968,               // LOG
  ITFILTTABLE = 1991,              // LOG
  ITBANDPASSCONFIG = 1999,         // COMMAND
  ITPROGFILTCONFIG = 2000,         // COMMAND
  ITBANDPASSBANK = 2022,           // LOG
  ITPROGFILTBANK = 2023,           // LOG
  ITFRONTENDMODE = 2039,           // COMMAND
  RANGECMP4 = 2050,                // LOG
  ITPSDDETECT = 2063,              // LOG
  ITDETECTSTATUS = 2065,           // LOG
  NAVICRAWSUBFRAME = 2105,         // LOG
  NAVICALMANAC = 2122,             // LOG
  NAVICEPHEMERIS = 2123,           // LOG
  NAVICIONO = 2124,                // LOG
  NAVICSYSCLOCK = 2125,            // LOG
  NAVICECUTOFF = 2134,             // COMMAND
  ITDETECTCONFIG = 2143,           // COMMAND
  RAWSBASFRAME2 = 2185,            // LOG
  FORCEGALE6CODE = 2222,           // COMMAND
  GALCNAVRAWPAGE = 2239,           // LOG
  QZSSCNAVRAWMESSAGE = 2261,       // LOG
  GPSCNAVRAWMESSAGE = 2262,        // LOG
  SPRINKLERDATA = 2274,            // LOG
  SPRINKLERCONFIG = 2275,          // COMMAND
  SPRINKLERDATAH = 2276,           // LOG
  ITWARNINGCONFIG = 2289,          // COMMAND
  TRACKSIGNAL = 2311,              // COMMAND
  SKDETECTCONFIG = 2358,           // COMMAND
  SKCALIBRATESTATUS = 2364,        // LOG
  SKCALIBRATE = 2366,              // COMMAND
  BDSBCNAV1EPHEMERIS = 2371,       // LOG
  BDSBCNAV2EPHEMERIS = 2372,       // LOG
  BDSBCNAV1RAWMESSAGE = 2373,      // LOG
  BDSBCNAV2RAWMESSAGE = 2374,      // LOG
  BDSBCNAV3RAWMESSAGE = 2411,      // LOG
  BDSBCNAV3EPHEMERIS = 2412,       // LOG
  LOCKOUTSATELLITE = 2415,         // COMMAND
  UNLOCKOUTSATELLITE = 2416,       // COMMAND
  INSATT = 263,                    // LOG
  INSPOS = 265,                    // LOG
  INSSPD = 266,                    // LOG
  INSVEL = 267,                    // LOG
  RAWIMU = 268,                    // LOG
  INSATTS = 319,                   // LOG
  INSPOSS = 321,                   // LOG
  INSSPDS = 323,                   // LOG
  INSVELS = 324,                   // LOG
  RAWIMUS = 325,                   // LOG
  INSCOMMAND = 379,                // COMMAND
  INSZUPT = 382,                   // COMMAND
  INSPVA = 507,                    // LOG
  INSPVAS = 508,                   // LOG
  CORRIMUDATA = 812,               // LOG
  CORRIMUDATAS = 813,              // LOG
  SETINITAZIMUTH = 863,            // COMMAND
  MARK1PVA = 1067,                 // LOG
  MARK2PVA = 1068,                 // LOG
  MARK3PVA = 1118,                 // LOG
  MARK4PVA = 1119,                 // LOG
  ALIGNMENTMODE = 1214,            // COMMAND
  TILTDATA = 1220,                 // LOG
  TILTFILTER = 1221,               // COMMAND
  TILTZERO = 1222,                 // COMMAND
  TAGNEXTMARK = 1257,              // COMMAND
  TAGGEDMARK1PVA = 1258,           // LOG
  TAGGEDMARK2PVA = 1259,           // LOG
  SETIMUSPECS = 1295,              // COMMAND
  IMURATEPVAS = 1305,              // LOG
  INPUTGIMBALANGLE = 1317,         // COMMAND
  VARIABLELEVERARM = 1320,         // LOG
  GIMBALLEDPVA = 1321,             // LOG
  TAGGEDMARK3PVA = 1327,           // LOG
  TAGGEDMARK4PVA = 1328,           // LOG
  SETUPSENSOR = 1333,              // COMMAND
  TIMEDEVENTPULSE = 1337,          // COMMAND
  IMURATECORRIMUS = 1362,          // LOG
  HEAVE = 1382,                    // LOG
  SETHEAVEWINDOW = 1383,           // COMMAND
  SETALIGNMENTVEL = 1397,          // COMMAND
  CONNECTIMU = 1428,               // COMMAND
  RELINSPVA = 1446,                // LOG
  INSTHRESHOLDS = 1448,            // COMMAND
  TSS1 = 1456,                     // LOG
  INSATTX = 1457,                  // LOG
  INSVELX = 1458,                  // LOG
  INSPOSX = 1459,                  // LOG
  RAWIMUX = 1461,                  // LOG
  RAWIMUSX = 1462,                 // LOG
  EXTERNALPVAS = 1463,             // LOG
  INSPVAX = 1465,                  // LOG
  SYNCHEAVE = 1708,                // LOG
  DELAYEDHEAVE = 1709,             // LOG
  SYNCRELINSPVA = 1743,            // LOG
  RELINSAUTOMATION = 1763,         // COMMAND
  SETIMUPORTPROTOCOL = 1767,       // COMMAND
  SETRELINSOUTPUTFRAME = 1775,     // COMMAND
  IMURATEPVA = 1778,               // LOG
  RELINSCONFIG = 1797,             // COMMAND
  SETMAXALIGNMENTTIME = 1800,      // COMMAND
  SETINSUPDATE = 1821,             // COMMAND
  INSUPDATESTATUS = 1825,          // LOG
  INSCALIBRATE = 1882,             // COMMAND
  INSPVASDCMP = 1890,              // LOG
  INSSEED = 1906,                  // COMMAND
  SETINSTRANSLATION = 1920,        // COMMAND
  SETINSROTATION = 1921,           // COMMAND
  SETINSPROFILE = 1944,            // COMMAND
  INSCONFIG = 1945,                // LOG
  INSCALSTATUS = 1961,             // LOG
  SETIMUEVENT = 1965,              // COMMAND
  INSSTDEV = 2051,                 // LOG
  INSSTDEVS = 2052,                // LOG
  INSATTQS = 2118,                 // LOG
  INSSEEDSTATUS = 2129,            // LOG
  INSALIGNCONFIG = 2163,           // COMMAND
  SETALIGNMENTORIENTATION = 2164,  // COMMAND
  CORRIMUS = 2264,                 // LOG
  RAWDMI = 2269,                   // COMMAND
  DMICONFIG = 2270,                // COMMAND
  TILTSTATUS = 2310,               // LOG
  INSVELUSER = 2318,               // LOG
  INSDATUMINFO = 2384,             // LOG
  CLOCKMODEL = 16,                 // LOG
  BESTPOS = 42,                    // LOG
  FIX = 44,                        // COMMAND
  PSRPOS = 47,                     // LOG
  USERDATUM = 78,                  // COMMAND
  RTKELEVMASK = 91,                // COMMAND
  RTKSVENTRIES = 92,               // COMMAND
  MATCHEDPOS = 96,                 // LOG
  RTKCOMMAND = 97,                 // COMMAND
  BESTVEL = 99,                    // LOG
  PSRVEL = 100,                    // LOG
  SETRTCM16 = 131,                 // COMMAND
  RTKPOS = 141,                    // LOG
  DGPSTXID = 144,                  // COMMAND
  DATUM = 160,                     // COMMAND
  NAVIGATE = 161,                  // LOG
  SETNAV = 162,                    // COMMAND
  AVEPOS = 172,                    // LOG
  POSAVE = 173,                    // COMMAND
  PSRDOP = 174,                    // LOG
  REFSTATION = 175,                // LOG
  MAGVAR = 180,                    // COMMAND
  MARKPOS = 181,                   // LOG
  RTKDYNAMICS = 183,               // COMMAND
  UNDULATION = 214,                // COMMAND
  RTKVEL = 216,                    // LOG
  BESTXYZ = 241,                   // LOG
  MATCHEDXYZ = 242,                // LOG
  PSRXYZ = 243,
  RTKXYZ = 244,                          // LOG
  SETAPPROXPOS = 377,                    // COMMAND
  PDPFILTER = 424,                       // COMMAND
  PDPPOS = 469,                          // LOG
  PDPVEL = 470,                          // LOG
  PDPXYZ = 471,                          // LOG
  PSRDIFFSOURCE = 493,                   // COMMAND
  RTKSOURCE = 494,                       // COMMAND
  POSTIMEOUT = 612,                      // COMMAND
  MARK2POS = 615,                        // LOG
  SBASCONTROL = 652,                     // COMMAND
  BSLNXYZ = 686,                         // LOG
  SETDIFFCODEBIASES = 687,               // COMMAND
  GGAQUALITY = 691,                      // COMMAND
  SETIONOTYPE = 711,                     // COMMAND
  BESTUTM = 726,                         // LOG
  UTMZONE = 749,                         // COMMAND
  MOVINGBASESTATION = 763,               // COMMAND
  USEREXPDATUM = 783,                    // COMMAND
  SETBESTPOSCRITERIA = 839,              // COMMAND
  RTKQUALITYLEVEL = 844,                 // COMMAND
  RTKANTENNA = 858,                      // COMMAND
  LOCKOUTSYSTEM = 871,                   // COMMAND
  SETRTCM36 = 880,                       // COMMAND
  PSRTIME = 881,                         // LOG
  UNLOCKOUTSYSTEM = 908,                 // COMMAND
  RTKTIMEOUT = 910,                      // COMMAND
  DIFFCODEBIASCONTROL = 913,             // COMMAND
  RTKNETWORK = 951,                      // COMMAND
  RTKDOP = 952,                          // LOG
  PDPMODE = 970,                         // COMMAND
  HEADING = 971,                         // LOG
  SBASTIMEOUT = 1001,                    // COMMAND
  MASTERPOS = 1051,                      // LOG
  ROVERPOS = 1052,                       // LOG
  HDTOUTTHRESHOLD = 1062,                // COMMAND
  HEADINGOFFSET = 1082,                  // COMMAND
  HEADINGEXT = 1132,                     // LOG
  SETROVERID = 1135,                     // COMMAND
  OUTPUTDATUM = 1144,                    // COMMAND
  PSRSATS = 1162,                        // LOG
  PSRDOP2 = 1163,                        // LOG
  RTKDOP2 = 1172,                        // LOG
  RTKSATS = 1174,                        // LOG
  MATCHEDSATS = 1176,                    // LOG
  BESTSATS = 1194,                       // LOG
  IONOCONDITION = 1215,                  // COMMAND
  SETRTCMRXVERSION = 1216,               // COMMAND
  PDPSATS = 1234,                        // LOG
  GENERATERTKCORRECTIONS = 1260,         // COMMAND
  RAIMMODE = 1285,                       // COMMAND
  RAIMSTATUS = 1286,                     // LOG
  GENERATEDIFFCORRECTIONS = 1296,        // COMMAND
  ALIGNBSLNXYZ = 1314,                   // LOG
  ALIGNBSLNENU = 1315,                   // LOG
  HEADINGSATS = 1316,                    // LOG
  SETRTCMTXVERSION = 1322,               // COMMAND
  ALIGNAUTOMATION = 1323,                // COMMAND
  REFSTATIONINFO = 1325,                 // LOG
  ALIGNDOP = 1332,                       // LOG
  HEADING2 = 1335,                       // LOG
  GENERATEALIGNCORRECTIONS = 1349,       // LOG
  SETBASERECEIVERTYPE = 1374,            // COMMAND
  BASEANTENNAPCO = 1415,                 // COMMAND
  BASEANTENNAPCV = 1416,                 // COMMAND
  THISANTENNAPCO = 1417,                 // COMMAND
  THISANTENNAPCV = 1418,                 // COMMAND
  BASEANTENNATYPE = 1419,                // COMMAND
  THISANTENNATYPE = 1420,                // COMMAND
  SBASALMANAC = 1425,                    // LOG
  BESTGNSSPOS = 1429,                    // LOG
  BESTGNSSVEL = 1430,                    // LOG
  SETTROPOMODEL = 1434,                  // COMMAND
  RTKSOURCETIMEOUT = 1445,               // COMMAND
  RTKMATCHEDTIMEOUT = 1447,              // COMMAND
  PSRDIFFSOURCETIMEOUT = 1449,           // COMMAND
  PSRDIFFTIMEOUT = 1450,                 // COMMAND
  STEADYLINE = 1452,                     // COMMAND
  PPPPOS = 1538,                         // LOG
  PPPSATS = 1541,                        // LOG
  PPPRESET = 1542,                       // COMMAND
  PPPXYZ = 1543,                         // LOG
  PPPSEED = 1544,                        // COMMAND
  PPPDOP2 = 1546,                        // LOG
  PPPDYNAMICS = 1551,                    // COMMAND
  PPPTIMEOUT = 1560,                     // COMMAND
  PPPCONVERGEDCRITERIA = 1566,           // COMMAND
  UALCONTROL = 1627,                     // COMMAND
  HEADINGEXT2 = 1661,                    // LOG
  BESTVELTYPE = 1678,                    // COMMAND
  HEADINGRATE = 1698,                    // LOG
  PPPSOURCE = 1707,                      // COMMAND
  LBANDBEAMTABLE = 1718,                 // LOG
  TERRASTARINFO = 1719,                  // LOG
  VERIPOSINFO = 1728,                    // LOG
  TERRASTARSTATUS = 1729,                // LOG
  VERIPOSSTATUS = 1730,                  // LOG
  MARK3POS = 1738,                       // LOG
  MARK4POS = 1739,                       // LOG
  GLIDEINITIALIZATIONPERIOD = 1760,      // COMMAND
  DUALANTENNAALIGN = 1761,               // COMMAND
  AUTOSURVEY = 1795,                     // COMMAND
  RTKPORTMODE = 1936,                    // COMMAND
  PPPBASICCONVERGEDCRITERIA = 1949,      // COMMAND
  SAVEDSURVEYPOSITIONS = 1951,           // LOG
  SURVEYPOSITION = 1952,                 // COMMAND
  RTKASSIST = 1985,                      // COMMAND
  PDPDOP2 = 1995,                        // LOG
  PPPDOP = 1997,                         // LOG
  PDPDOP = 1998,                         // LOG
  STEADYLINEDIFFERENTIALTIMEOUT = 2002,  // COMMAND
  RTKASSISTTIMEOUT = 2003,               // COMMAND
  REFERENCESTATIONTIMEOUT = 2033,        // COMMAND
  DUALANTENNAHEADING = 2042,             // LOG
  GPHDTDUALANTENNA = 2045,               // LOG
  RTKASSISTSTATUS = 2048,                // LOG
  RTKINTEGERCRITERIA = 2070,             // COMMAND
  PPPDYNAMICSEED = 2071,                 // COMMAND
  RTKRESET = 2082,                       // COMMAND
  OCEANIXINFO = 2159,                    // LOG
  OCEANIXSTATUS = 2160,                  // LOG
  BESTVELX = 2230,                       // LOG
  PPPSEEDAPPLICATIONSTATUS = 2250,       // LOG
  TECTONICSCOMPENSATIONSOURCE = 2290,    // COMMAND
  TECTONICSCOMPENSATION = 2291,          // LOG
  PPPDATUMINFO = 2293,                   // LOG
  PSRDATUMINFO = 2300,                   // LOG
  BESTGNSSDATUMINFO = 2302,              // LOG
  PDPDATUMINFO = 2303,                   // LOG
  RTKDATUMINFO = 2304,                   // LOG
  BESTDATUMINFO = 2305,                  // LOG
  TILTCOMPENSATIONCONTROL = 2309,        // COMMAND
  WIFIAPPASSKEY = 2090,                  // COMMAND
  WIFIAPCHANNEL = 2091,                  // COMMAND
  WIFIAPSETTINGS = 2093,                 // LOG
  WIFIAPIPCONFIG = 2096,                 // COMMAND
  MEDIAFORMAT = 2128,                    // COMMAND
  WIFIMODE = 2144,                       // COMMAND
  SATELCONTROL = 2203,                   // COMMAND
  SATELDETECT = 2204,                    // COMMAND
  SATELSTATUS = 2205,                    // LOG
  WIFIAPSSID = 2206,                     // COMMAND
  WIFISTATUS = 2207,                     // LOG
  WIFINETLIST = 2210,                    // LOG
  WIFINETCONFIG = 2213,                  // COMMAND
  WIFIALIGNAUTOMATION = 2214,            // COMMAND
  SATEL4CONFIG = 2215,                   // COMMAND
  SATEL4INFO = 2216,                     // LOG
  SATEL9CONFIG = 2217,                   // COMMAND
  SATEL9CONFIGL = 2218,                  // COMMAND
  SATEL9CONFIGN = 2219,                  // COMMAND
  SATEL9INFO = 2220,                     // LOG
  SATELSTARTUPDETECT = 2221,             // COMMAND
  BLUETOOTHCONTROL = 2265,               // COMMAND
  PASSETH1 = 1209,                       // LOG
  SBAS12 = 979,                          // LOG
  SBAS27 = 986,                          // LOG
  SBAS6 = 995,                           // LOG
  INSPVACMP = 1889,                      // LOG
  INSCOV = 264,                          // LOG
  INSCOVS = 320,                         // LOG
  SETMARK3OFFSET = 1116,                 // COMMAND
  SETMARK4OFFSET = 1117,                 // COMMAND
  SETIMUTOANTOFFSET = 383,               // COMMAND
  SETIMUORIENTATION = 567,               // COMMAND
  RVBCALIBRATE = 641,                    // COMMAND
  VEHICLEBODYROTATION = 642,             // COMMAND
  INSWHEELUPDATE = 647,                  // COMMAND
  BESTLEVERARM = 674,                    // LOG
  LEVERARMCALIBRATE = 675,               // COMMAND
  SETINSOFFSET = 676,                    // COMMAND
  INSUPDATE = 757,                       // LOG
  SETINITATTITUDE = 862,                 // COMMAND
  APPLYVEHICLEBODYROTATION = 1071,       // COMMAND
  EXTHDGOFFSET = 1204,                   // COMMAND
  SETIMUTOANTOFFSET2 = 1205,             // COMMAND
  BESTLEVERARM2 = 1256,                  // LOG
  IMUTOANTOFFSETS = 1270,                // LOG
  INSZUPTCONTROL = 1293,                 // COMMAND
  SETGIMBALORIENTATION = 1318,           // COMMAND
  GIMBALSPANROTATION = 1319,             // COMMAND
  SETIMUTOGIMBALOFFSET = 1352,           // COMMAND
  SETMARK1OFFSET = 1069,                 // COMMAND
  SETMARK2OFFSET = 1070,                 // COMMAND
  SETWHEELSOURCE = 1722,                 // COMMAND
  RAWLBANDFRAME = 732,                   // LOG
  RAWLBANDPACKET = 733,                  // LOG
  PDPVELOCITYOUT = 1324,                 // COMMAND
  DGPSEPHEMDELAY = 142,                  // COMMAND
  GPGGARTK = 259,                        // LOG
  LBANDINFO = 730,                       // LOG
  BASEANTENNAMODEL = 870,                // COMMAND
  SBAS32 = 988,                          // LOG
  SBAS33 = 989,                          // LOG
  SBAS34 = 990,                          // LOG
  SBAS35 = 991,                          // LOG
  SBAS45 = 993,                          // LOG
  SETCANNAME = 1091,                     // COMMAND
  INVALID = 0xFFFF
};
}
