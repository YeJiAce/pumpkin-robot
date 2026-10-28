"""南瓜机器人 AstrBot 插件:
1) 导入 pumpkin_platform 注册"pumpkin_voice"平台(本机 HTTP 消息入口)
2) 给 LLM 注册控车工具:move / stop / say / follow,全部转发给宿主机运动桥(127.0.0.1:8123)
3) pumpkin_look 看图工具:桥抓一帧摄像头 → 旁路发给 deepseek(网关里唯一能看图的)
   → 描述文字还给主模型。限幅由运动桥(pumpkin_voice 侧)强制执行,插件不重复实现。"""
import json

import aiohttp

from astrbot import logger
from astrbot.api.event import AstrMessageEvent, filter
from astrbot.api.star import Context, Star, register

BRIDGE = "http://127.0.0.1:8123"      # 宿主机运动桥(pumpkin_voice 里的线程)

GATEWAY_URL = "https://api.lan.x-lab.cc/v1/chat/completions"
VISION_MODEL = "glm-5.3-flash"   # 10-27 实测网关能看图;看图=high 档,开着思考描述更准
VISION_MAX_TOKENS = 1500


@register("astrbot_plugin_pumpkin", "yjy", "南瓜机器人语音入口与运动工具", "1.0.0")
class PumpkinPlugin(Star):
    def __init__(self, context: Context):
        super().__init__(context)
        from . import pumpkin_platform  # noqa: F401  导入即注册平台适配器

    async def _bridge_json(self, path: str, payload: dict,
                           timeout: float = 8) -> dict:
        """调宿主机桥,原样返回 JSON。"""
        async with aiohttp.ClientSession() as s:
            async with s.post(BRIDGE + path, json=payload,
                              timeout=aiohttp.ClientTimeout(total=timeout)) as r:
                return await r.json()

    async def _bridge(self, path: str, payload: dict, timeout: float = 8) -> str:
        """调运动桥,把结果变成给模型看的一句话。桥挂了模型也知道车没响应。"""
        try:
            res = await self._bridge_json(path, payload, timeout)
            if res.get("ok"):
                return res.get("echo", "已执行")
            return "执行失败:" + str(res.get("error", "未知原因"))
        except Exception as e:
            return f"联系不上车(运动桥无响应): {type(e).__name__}"

    def _gateway_key(self) -> str:
        """看图要直连网关,key 复用 AstrBot 自己配的(不重复填)。"""
        if getattr(self, "_gw_key", None):
            return self._gw_key
        try:
            with open("/AstrBot/data/cmd_config.json", encoding="utf-8-sig") as f:
                srcs = json.load(f).get("provider_sources", [])
            k = srcs[0].get("key")
            self._gw_key = k[0] if isinstance(k, list) else k
        except Exception as e:
            logger.warning(f"读网关 key 失败: {e}")
            self._gw_key = ""
        return self._gw_key

    async def _ask_vision(self, b64: str, question: str) -> str:
        """把一帧画面发给 deepseek 看,返回它的一段描述文字。"""
        key = self._gateway_key()
        if not key:
            return "看图服务没配好(读不到网关 key)"
        body = {
            "model": VISION_MODEL, "max_tokens": VISION_MAX_TOKENS,
            "thinking": {"type": "enabled"},   # 看图用 high 档(网关认这个字段,实测)
            "messages": [{"role": "user", "content": [
                {"type": "text",
                 "text": f"{question}(你是机器人的眼睛,客观描述画面,两三句话)"},
                {"type": "image_url",
                 "image_url": {"url": "data:image/jpeg;base64," + b64}},
            ]}],
        }
        for attempt in (1, 2):                    # 空回复(思考吃光token)重试一次
            try:
                async with aiohttp.ClientSession() as s:
                    async with s.post(GATEWAY_URL, json=body, headers={
                            "Authorization": "Bearer " + key},
                            timeout=aiohttp.ClientTimeout(total=60)) as r:
                        d = await r.json()
                text = (d["choices"][0]["message"].get("content") or "").strip()
                if text:
                    return text
                logger.warning(f"看图第{attempt}次空回复,重试")
            except Exception as e:
                logger.warning(f"看图调用失败({attempt}): {type(e).__name__} {e}")
                if attempt == 2:
                    return f"看图失败({type(e).__name__})"
        return "刚才没看清,再问一次试试"

    @filter.llm_tool("pumpkin_look")
    async def pumpkin_look(self, event: AstrMessageEvent, question: str) -> str:
        """用摄像头看一眼当前画面,再回答任何关于"现在"的视觉问题。
        凡是问镜头前的/你现在看到的/主人现在长相穿着在干嘛/周围环境,都必须先调用本工具看了再说,
        不许凭空想象。也用于点评、吐槽、夸奖镜头前的人这类需要真实画面的请求。

        Args:
            question (string): 想从画面里了解什么,如"描述画面里的人,包括穿着和动作"
        """
        logger.info(f"pumpkin_look: {question}")
        try:
            snap = await self._bridge_json("/snap", {}, timeout=10)
        except Exception as e:
            return f"眼睛(摄像头)没响应: {type(e).__name__}"
        if not snap.get("ok"):
            return "眼睛(摄像头)打不开:" + str(snap.get("error"))
        desc = await self._ask_vision(snap.get("b64", ""), question)
        logger.info(f"pumpkin_look 结果: {desc[:80]}")
        return desc

    @filter.llm_tool("pumpkin_move")
    async def pumpkin_move(self, event: AstrMessageEvent, segments: str) -> str:
        """让南瓜机器人移动。动作段全部是闭环定量段,底盘靠里程计精确走到位,只给意图值。
        硬性流程:调本工具之前必须先调一次 pumpkin_say 说开场白——跳舞表演说俏皮的(如"看好啦,我要跳啦!"),普通指令简短应一声(如"收到,走喽!")。
        动作真正执行完(最长约20秒)后本工具才返回,返回后即可向用户汇报结果。

        Args:
            segments (string): 动作段的JSON数组字符串,只有四种段:转角度 {"mode":"turn","angle":90}(度,左转为正,±360内,多圈拆多段);走距离 {"mode":"move","dist":0.3}(米,前进为正,±0.8内);横移 {"mode":"strafe","dist":0.3}(米,左移为正,±0.8内);回原位 {"mode":"home"}(自动精确回到本次动作开始时的位置和朝向)。跳舞=自主组合 turn/move/strafe 编走位(方形/三角/画圈,每次不同),结尾必须加 home 回原位。没有速度×时长段;用户说了角度/距离/圈数直接用对应段,不要换算(「转一圈」=turn 360)。
        """
        logger.info(f"pumpkin_move: {segments[:200]}")
        return await self._bridge("/move", {"segments": segments}, timeout=75)

    @filter.llm_tool("pumpkin_say")
    async def pumpkin_say(self, event: AstrMessageEvent, text: str) -> str:
        """让南瓜立刻开口说一句话(交给语音队列播报,与动作并行互不影响)。
        用途是动作前的开场白:先调本工具说一句,紧接着再调 pumpkin_move 开始动作。
        纯聊天不要用本工具——直接文字回复就会自动播报。

        Args:
            text (string): 要说的话,一句,不超过20字,纯文字不带表情符号
        """
        logger.info(f"pumpkin_say: {text[:50]}")
        return await self._bridge("/say", {"text": text}, timeout=5)

    @filter.llm_tool("pumpkin_stop")
    async def pumpkin_stop(self, event: AstrMessageEvent) -> str:
        """立即急停南瓜机器人,取消所有进行中的动作。"""
        logger.info("pumpkin_stop")
        return await self._bridge("/stop", {})

    @filter.llm_tool("pumpkin_follow")
    async def pumpkin_follow(self, event: AstrMessageEvent, action: str) -> str:
        """启动或停止视觉跟随模式(跟着人走)。

        Args:
            action (string): start(开始跟随) 或 stop(停止跟随)
        """
        logger.info(f"pumpkin_follow: {action}")
        return await self._bridge("/follow", {"action": action})
