"""南瓜语音事件:管线发来的每一条消息都转发给宿主机音箱(8123 /say)排队播放,
一次交互可以有多条(工具前预告 + 工具后汇报),这是 AstrBot 标准的 event.send 扩展点。
第一条消息同时回填 Future,平台据此知道"开始有回复了"。"""
import asyncio

import aiohttp

from astrbot.api.event import AstrMessageEvent, MessageChain
from astrbot.api.message_components import Plain

SAY_URL = "http://127.0.0.1:8123/say"


class PumpkinVoiceEvent(AstrMessageEvent):
    """语音端事件。reply_future / last_send_ts 由平台适配器创建事件时挂上:
    - reply_future:第一次 send 即 set,/chat 端确认"大脑开口了"
    - last_send_ts:最近一次 send 的时刻,/chat 端据此等安静窗口判"说完了没"
    """

    reply_future: asyncio.Future
    last_send_ts: float

    async def send(self, message: MessageChain):
        text = "".join(c.text for c in message.chain if isinstance(c, Plain))
        if text:
            self.last_send_ts = asyncio.get_running_loop().time()
            if not self.reply_future.done():
                self.reply_future.set_result(text)
            try:
                # 推给音箱排队逐条播;推不动也不抛(语音进程没跑时 /chat 还能带文本兜底)
                async with aiohttp.ClientSession() as s:
                    async with s.post(SAY_URL, json={"text": text},
                                      timeout=aiohttp.ClientTimeout(total=5)):
                        pass
            except Exception:
                pass
        await super().send(message)

    async def send_streaming(self, *args, **kwargs):
        """语音端不需要流式,等完整消息逐条播即可。"""
        pass
