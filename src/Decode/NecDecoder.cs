using System;
using System.Collections;
using nanoFramework.Hardware.Esp32.Rmt;

namespace nF.Irc.Decode
{
    public class NecDecoder
    {
        const int NEC_DATA_ITEM_NUM = 34;                               /*!< NEC code item number: header + 32bit data + end */
        const int NEC_HEADER_HIGH_US = 9000;                            /*!< NEC protocol header: positive 9ms */
        const int NEC_HEADER_LOW_US = 4500;                             /*!< NEC protocol header: negative 4.5ms*/
        const int NEC_BIT_ONE_HIGH_US = 560;                            /*!< NEC protocol data bit 1: positive 0.56ms */
        const int NEC_BIT_ONE_LOW_US = (2250 - NEC_BIT_ONE_HIGH_US);    /*!< NEC protocol data bit 1: negative 1.69ms */
        const int NEC_BIT_ZERO_HIGH_US = 560;                           /*!< NEC protocol data bit 0: positive 0.56ms */
        const int NEC_BIT_ZERO_LOW_US = (1120 - NEC_BIT_ZERO_HIGH_US);  /*!< NEC protocol data bit 0: negative 0.56ms */
        const int NEC_BIT_END = 560;                                    /*!< NEC protocol end: positive 0.56ms */
        const int NEC_BIT_MARGIN = 400;                                 /*!< NEC parse margin time */
        const int NEC_BIT_MARGIN_HDR = 2000;
        const bool RMT_RX_ACTIVE_LEVEL = false;

        readonly int _tick_10_us;

        public NecDecoder(int tick_10_us)
        {
            _tick_10_us = tick_10_us;
        }

        public bool TryDecode(RmtCommand[] items, out uint address, out uint data)
        {
            address = data = uint.MinValue;

            if (items.Length < NEC_DATA_ITEM_NUM || !NEC_HeaderIf(items[0]))
                return false;

            uint addr = 0, notAddr = 0, cmd = 0, notCmd = 0, accumValue = 0;
            int accumCounter = 0;

            for(int i = 1; i < NEC_DATA_ITEM_NUM; i++)
            {
                if (NEC_BitOneIf(items[i]))
                {
                    accumValue = (accumValue >> 1) | 0x80;
                }
                else if (NEC_BitZeroIf(items[i]))
                {
                    accumValue = accumValue >> 1;
                }

                if (accumCounter == 7)
                {
                    accumCounter = 0;
                    if (i == 8)
                        addr = accumValue;       // 地址
                    else if (i == 16)
                        notAddr = accumValue;    // 地址反码
                    else if (i == 24)
                        cmd = accumValue;           // 数据
                    else if (i == 32)
                        notCmd = accumValue;        // 数据反码

                    accumValue = 0;
                }
                else
                {
                    accumCounter++;
                }
            }

            // 校验
            if (addr != (notAddr ^ 0xff) || cmd != (notCmd ^ 0xff))
                return false;

            address = addr;
            data = cmd;

            return true;
        }

        bool NEC_CheckInRange(int duration_ticks, int target_us, int margin_us)
        {
            if ((NEC_ITEM_DURATION(duration_ticks) < (target_us + margin_us))
                && (NEC_ITEM_DURATION(duration_ticks) > (target_us - margin_us)))
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        bool NEC_HeaderIf(RmtCommand item)
        {
            if ((item.Level0 == RMT_RX_ACTIVE_LEVEL && item.Level1 != RMT_RX_ACTIVE_LEVEL)
                && NEC_CheckInRange(item.Duration0, NEC_HEADER_HIGH_US, NEC_BIT_MARGIN_HDR)
                && NEC_CheckInRange(item.Duration1, NEC_HEADER_LOW_US, NEC_BIT_MARGIN_HDR))
            {
                return true;
            }
            return false;
        }

        bool NEC_BitOneIf(RmtCommand item)
        {
            if ((item.Level0 == RMT_RX_ACTIVE_LEVEL && item.Level1 != RMT_RX_ACTIVE_LEVEL)
                && NEC_CheckInRange(item.Duration0, NEC_BIT_ONE_HIGH_US, NEC_BIT_MARGIN)
                && NEC_CheckInRange(item.Duration1, NEC_BIT_ONE_LOW_US, NEC_BIT_MARGIN))
            {
                return true;
            }
            return false;
        }

        bool NEC_BitZeroIf(RmtCommand item)
        {
            if ((item.Level0 == RMT_RX_ACTIVE_LEVEL && item.Level1 != RMT_RX_ACTIVE_LEVEL)
                && NEC_CheckInRange(item.Duration0, NEC_BIT_ZERO_HIGH_US, NEC_BIT_MARGIN)
                && NEC_CheckInRange(item.Duration1, NEC_BIT_ZERO_LOW_US, NEC_BIT_MARGIN))
            {
                return true;
            }
            return false;
        }

        int NEC_ITEM_DURATION(int d)
        {
            return (d & 0x7fff) * 10 / _tick_10_us;
        }
    }
}
