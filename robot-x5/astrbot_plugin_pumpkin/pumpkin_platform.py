"""南瓜语音平台适配器:本机 127.0.0.1:6186 开一个 HTTP 口,
pumpkin_voice(或任何本机程序)POST /chat {"text": "..."} 即等于"说了一句话"。
回复不走主动推送,等管线 send 后经 Future 从 HTTP 响应带回。"""
import asyncio
import uuid

from astrbot import logger
from astrbot.api.event import MessageChain
from astrbot.api.message_components import Image, Plain
from astrbot.api.platform import (
    AstrBotMessage,
    MessageMember,
    MessageType,
    Platform,
    PlatformMetadata,
    register_platform_adapter,
)
from astrbot.core.platform.astr_message_event import MessageSesion
from astrbot.core.platform.webhook_server import FastAPIWebhookServer

from .pumpkin_event import PumpkinVoiceEvent


@register_platform_adapter(
    "pumpkin_voice",
    "南瓜机器人语音入口(本机HTTP)",
    default_config_tmpl={"port": 6186},
)
class PumpkinVoicePlatform(Platform):
    def __init__(self, platform_config: dict, platform_settings: dict,
                 event_queue: asyncio.Queue) -> None:
        super().__init__(platform_config, event_queue)
        self.settings = platform_settings
        self.port = int(platform_config.get("port", 6186))
        self._server = FastAPIWebhookServer("pumpkin_voice")
        self._server.route("/chat", methods=["POST"])(self._chat_route)
        self._shutdown = asyncio.Event()

    def meta(self) -> PlatformMetadata:
        return PlatformMetadata("pumpkin_voice", "南瓜机器人语音入口", "pumpkin_voice")

    async def send_by_session(self, session: MessageSesion,
                              message_chain: MessageChain) -> None:
        # 语音端回复全部走 HTTP 请求-响应,不需要主动推送通道
        pass

    async def run(self):
        logger.info(f"南瓜语音入口: http://127.0.0.1:{self.port}/chat")
        await self._server.run_task(host="127.0.0.1", port=self.port,
                                    shutdown_trigger=self._shutdown.wait)

    async def terminate(self):
        self._shutdown.set()

    # ---------- HTTP -> 事件 ----------

    async def _chat_route(self, req):
        data = await req.get_json(silent=True)
        if not isinstance(data, dict):
            return {"ok": False, "reply": "请求体不是JSON"}
        return await self.handle_chat(data)

    async def handle_chat(self, data: dict) -> dict:
        text = str(data.get("text") or "").strip()
        if not text:
            return {"ok": False, "reply": "text为空"}
        images = [b for b in data.get("images") or [] if isinstance(b, str)][:3]

        abm = AstrBotMessage()
        abm.type = MessageType.FRIEND_MESSAGE      # 私聊语义:免唤醒词,直接进管线
        abm.message_str = text
        abm.sender = MessageMember(user_id="voice", nickname="主人")
        abm.message = [Plain(text=text)]
        for b64 in images:                          # 二期视觉:glm-5.3不收图,先带上,路由到视觉模型是后话
            abm.message.append(
                Image(file=b64 if b64.startswith(("data:", "http", "base64:"))
                      else "base64://" + b64))
        abm.raw_message = {"text": text, "n_images": len(images)}
        abm.self_id = "pumpkin"
        abm.session_id = "voice"
        abm.message_id = uuid.uuid4().hex

        ev = PumpkinVoiceEvent(message_str=text, message_obj=abm,
                               platform_meta=self.meta(), session_id="voice")
        ev.reply_future = asyncio.get_running_loop().create_future()
        ev.last_send_ts = None
        self.commit_event(ev)

        # 一场交互可能有多条消息(预告/汇报),逐条推给音箱;/chat 等最后一条
        # 之后安静 QUIET 秒才算"说完了"再返回,语音进程据此知道何时收工。
        timeout = float(data.get("timeout", 120))
        quiet = 3.0
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout
        try:
            await asyncio.wait_for(ev.reply_future, timeout=timeout)
            while loop.time() < deadline:
                since = loop.time() - ev.last_send_ts
                if since >= quiet:
                    break
                await asyncio.sleep(min(0.5, max(0.05, quiet - since)))
            return {"ok": True, "reply": ev.reply_future.result()}
        except asyncio.TimeoutError:
            return {"ok": False, "reply": "(大脑想得太久,超时了)"}
        except Exception as e:
            return {"ok": False, "reply": f"(出错了: {e})"}
