#include "lunar.h"
#include <string.h>

const char Lunar_MonthString[13][7] = {
    "----", "正月", "二月", "三月", "四月", "五月", "六月",
    "七月", "八月", "九月", "十月", "冬月", "腊月"
};

const char Lunar_MonthLeapString[2][4] = {" ", "闰"};

const char Lunar_DateString[31][7] = {
    "----", "初一", "初二", "初三", "初四", "初五", "初六", "初七",
    "初八", "初九", "初十", "十一", "十二", "十三", "十四", "十五",
    "十六", "十七", "十八", "十九", "二十", "廿一", "廿二", "廿三",
    "廿四", "廿五", "廿六", "廿七", "廿八", "廿九", "三十"
};

const char Lunar_DayString[7][4] = {"日", "一", "二", "三", "四", "五", "六"};

const char Lunar_ZodiacString[12][4] = {
    "猴", "鸡", "狗", "猪", "鼠", "牛", "虎", "兔", "龙", "蛇", "马", "羊"
};

const char Lunar_StemStrig[10][4] = {
    "庚", "辛", "壬", "癸", "甲", "乙", "丙", "丁", "戊", "己"
};

const char Lunar_BranchStrig[12][4] = {
    "申", "酉", "戌", "亥", "子", "丑", "寅", "卯", "辰", "巳", "午", "未"
};

/* 1997 ~ 2049 */
static const uint32_t lunar_month_days[] = {
    1997,
    0x0000B26D, 0x0000125C, 0x0000192C, 0x00009A95, 0x00001A94, 0x00001B4A,
    0x00004B55, 0x00000AD4, 0x0000F55B, 0x000004BA, 0x0000125A, 0x0000B92B,
    0x0000152A, 0x00001694, 0x000096AA, 0x000015AA, 0x00012AB5, 0x00000974,
    0x000014B6, 0x0000CA57, 0x00000A56, 0x00001526, 0x00008E95, 0x00000D54,
    0x000015AA, 0x000049B5, 0x0000096C, 0x0000D4AE, 0x0000149C, 0x00001A4C,
    0x0000BD26, 0x00001AA6, 0x00000B54, 0x00006D6A, 0x000012DA, 0x0001695D,
    0x0000095A, 0x0000149A, 0x0000DA4B, 0x00001A4A, 0x00001AA4, 0x0000BB54,
    0x000016B4, 0x00000ADA, 0x0000495B, 0x00000936, 0x0000F497, 0x00001496,
    0x0000154A, 0x0000B6A5, 0x00000DA4, 0x000015B4, 0x00006AB6, 0x0000126E,
};

/* 1997 ~ 2049 */
static const uint32_t solar_1_1[] = {
    1997,
    0x000F9C3C, 0x000F9E50, 0x000FA045, 0x000FA238, 0x000FA44C, 0x000FA641,
    0x000FA836, 0x000FAA49, 0x000FAC3D, 0x000FAE52, 0x000FB047, 0x000FB23A,
    0x000FB44E, 0x000FB643, 0x000FB837, 0x000FBA4A, 0x000FBC3F, 0x000FBE53,
    0x000FC048, 0x000FC23C, 0x000FC450, 0x000FC645, 0x000FC839, 0x000FCA4C,
    0x000FCC41, 0x000FCE36, 0x000FD04A, 0x000FD23D, 0x000FD451, 0x000FD646,
    0x000FD83A, 0x000FDA4D, 0x000FDC43, 0x000FDE37, 0x000FE04B, 0x000FE23F,
    0x000FE453, 0x000FE648, 0x000FE83C, 0x000FEA4F, 0x000FEC44, 0x000FEE38,
    0x000FF04C, 0x000FF241, 0x000FF436, 0x000FF64A, 0x000FF83E, 0x000FFA51,
    0x000FFC46, 0x000FFE3A, 0x0010004E, 0x00100242, 0x00100437, 0x0010064B,
};

static uint32_t GetBitInt(uint32_t data, uint8_t length, uint8_t shift) {
    return (data & (((1 << length) - 1) << shift)) >> shift;
}

static uint16_t SolarToInt(uint16_t y, uint8_t m, uint8_t d) {
    m = (m + 9) % 12;
    y = y - m / 10;
    return 365 * y + y / 4 - y / 100 + y / 400 + (m * 306 + 5) / 10 + (d - 1);
}

void LUNAR_SolarToLunar(struct Lunar_Date *lunar, uint16_t solar_year,
                         uint8_t solar_month, uint8_t solar_date) {
    uint8_t i, lunarM, m, d, leap, dm;
    uint16_t year_index, lunarY, y, offset;
    uint32_t solar_data, solar11, days;

    if (solar_month < 1 || solar_month > 12 || solar_date < 1 || solar_date > 31 ||
        (solar_year - solar_1_1[0] < 3) ||
        ((solar_year - solar_1_1[0]) > (sizeof(solar_1_1) / sizeof(uint32_t) - 2))) {
        lunar->Year = 0;
        lunar->Month = 0;
        lunar->Date = 0;
        lunar->IsLeap = 0;
        return;
    }

    year_index = solar_year - solar_1_1[0];
    solar_data = ((uint32_t)solar_year << 9) | ((uint32_t)solar_month << 5) | ((uint32_t)solar_date);
    if (solar_1_1[year_index] > solar_data) {
        year_index -= 1;
    }
    solar11 = solar_1_1[year_index];
    y = GetBitInt(solar11, 12, 9);
    m = GetBitInt(solar11, 4, 5);
    d = GetBitInt(solar11, 5, 0);
    offset = SolarToInt(solar_year, solar_month, solar_date) - SolarToInt(y, m, d);

    days = lunar_month_days[year_index];
    leap = GetBitInt(days, 4, 13);

    lunarY = year_index + solar_1_1[0];
    lunarM = 1;
    offset += 1;
    for (i = 0; i < 13; i++) {
        if (GetBitInt(days, 1, 12 - i) == 1) {
            dm = 30;
        } else {
            dm = 29;
        }
        if (offset > dm) {
            lunarM += 1;
            offset -= dm;
        } else {
            break;
        }
    }
    lunar->IsLeap = 0;
    if (leap != 0 && lunarM > leap) {
        if (lunarM == leap + 1) {
            lunar->IsLeap = 1;
        }
        lunarM -= 1;
    }
    lunar->Month = lunarM;
    lunar->Date = offset;
    lunar->Year = lunarY;
}

uint8_t LUNAR_GetZodiac(const struct Lunar_Date *lunar) { return lunar->Year % 12; }
uint8_t LUNAR_GetStem(const struct Lunar_Date *lunar) { return lunar->Year % 10; }
uint8_t LUNAR_GetBranch(const struct Lunar_Date *lunar) { return lunar->Year % 12; }

/* ---- 24 Solar Terms ---- */

static const uint8_t YearMonthBit[160] = {
    0x4E, 0xA6, 0x99,  // 2000
    0x9C, 0xA2, 0x98,  // 2001
    0x80, 0x00, 0x18,  // 2002
    0x00, 0x10, 0x24,  // 2003
    0x4E, 0xA6, 0x99,  // 2004
    0x9C, 0xA2, 0x98,  // 2005
    0x80, 0x82, 0x18,  // 2006
    0x00, 0x10, 0x24,  // 2007
    0x4E, 0xA6, 0xD9,  // 2008
    0x9E, 0xA2, 0x98,  // 2009
    0x80, 0x82, 0x18,  // 2010
    0x00, 0x10, 0x04,  // 2011
    0x4E, 0xE6, 0xD9,  // 2012
    0x9E, 0xA6, 0xA8,  // 2013
    0x80, 0x82, 0x18,  // 2014
    0x00, 0x10, 0x00,  // 2015
    0x0F, 0xE6, 0xD9,  // 2016
    0xBE, 0xA6, 0x98,  // 2017
    0x88, 0x82, 0x18,  // 2018
    0x80, 0x00, 0x00,  // 2019
    0x0F, 0xEF, 0xD9,  // 2020
    0xBE, 0xA6, 0x99,  // 2021
    0x8C, 0x82, 0x98,  // 2022
    0x80, 0x00, 0x00,  // 2023
    0x0F, 0xEF, 0xDB,  // 2024
    0xBE, 0xA6, 0x99,  // 2025
    0x9C, 0xA2, 0x98,  // 2026
    0x80, 0x00, 0x18,  // 2027
    0x0F, 0xEF, 0xDB,  // 2028
    0xBE, 0xA6, 0x99,  // 2029
    0x9C, 0xA2, 0x98,  // 2030
    0x80, 0x00, 0x18,  // 2031
    0x0F, 0xEF, 0xDB,  // 2032
    0xBE, 0xA2, 0x99,  // 2033
    0x8C, 0xA0, 0x98,  // 2034
    0x80, 0x82, 0x18,  // 2035
    0x0B, 0xEF, 0xDB,  // 2036
    0xBE, 0xA6, 0x99,  // 2037
    0x8C, 0xA2, 0x98,  // 2038
    0x80, 0x82, 0x18,  // 2039
    0x0F, 0xEF, 0xDB,  // 2040
    0xBE, 0xE6, 0xD9,  // 2041
    0x9E, 0xA2, 0x98,  // 2042
    0x80, 0x82, 0x18,  // 2043
    0x0F, 0xEF, 0xFB,  // 2044
    0xBF, 0xE6, 0xD9,  // 2045
    0x9E, 0xA6, 0x98,  // 2046
    0x80, 0x82, 0x18,  // 2047
    0x0F, 0xFF, 0xFF,  // 2048
    0xFC, 0xEF, 0xD9,  // 2049
    0xBE, 0xA6, 0x18,  // 2050
};

static const uint8_t solar_term_days[24] = {
    6, 20, 4, 19, 6, 21, 5, 20, 6, 21, 6, 21,
    7, 23, 8, 23, 8, 23, 8, 24, 8, 22, 7, 22,
};

const char JieQiStr[24][7] = {
    "小寒", "大寒", "立春", "雨水", "惊蛰", "春分",
    "清明", "谷雨", "立夏", "小满", "芒种", "夏至",
    "小暑", "大暑", "立秋", "处暑", "白露", "秋分",
    "寒露", "霜降", "立冬", "小雪", "大雪", "冬至",
};

uint8_t GetJieQi(uint16_t myear, uint8_t mmonth, uint8_t mday, uint8_t *JQdate) {
    uint8_t bak1, value, JQ;

    if ((myear < 2000) || (myear > 2050)) return 0;
    if ((mmonth == 0) || (mmonth > 12)) return 0;
    JQ = (mmonth - 1) * 2;
    if (mday >= 15) JQ++;

    bak1 = YearMonthBit[(myear - 2000) * 3 + JQ / 8];
    value = ((bak1 << (JQ % 8)) & 0x80);

    *JQdate = solar_term_days[JQ];
    if (value != 0) {
        if ((JQ == 1 || JQ == 11 || JQ == 18 || JQ == 21) && myear < 2044)
            (*JQdate)++;
        else
            (*JQdate)--;
    }
    return 1;
}

static const uint8_t MonthDayMax[12] = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
};

uint8_t GetJieQiStr(uint16_t myear, uint8_t mmonth, uint8_t mday, uint8_t *day) {
    uint8_t JQdate, JQ, MaxDay;

    if (GetJieQi(myear, mmonth, mday, &JQdate) == 0) return 0xFF;

    JQ = (mmonth - 1) * 2;
    if (mday >= 15) JQ++;

    if (mday == JQdate) {
        *day = 0;
        return JQ;
    }

    if (mday < JQdate) {
        mday = JQdate - mday;
    } else {
        JQ++;
        if (mday < 15) {
            GetJieQi(myear, mmonth, 15, &JQdate);
            mday = JQdate - mday;
        } else {
            MaxDay = MonthDayMax[mmonth - 1];
            if (mmonth == 2) {
                if ((myear % 4 == 0) && ((myear % 100 != 0) || (myear % 400 == 0))) MaxDay++;
            }
            if (++mmonth == 13) mmonth = 1;
            GetJieQi(myear, mmonth, 1, &JQdate);
            mday = MaxDay - mday + JQdate;
        }
    }
    *day = mday;
    return JQ;
}
