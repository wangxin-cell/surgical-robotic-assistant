"""Connect a spoken instrument ID to a fresh, read-only camera observation on the O6."""
import hashlib
from itertools import combinations
import json
import math
from pathlib import Path
import os
import tempfile

import cv2
import numpy as np
from PIL import Image, ImageDraw

from native_capture import camera_state, capture_scene, CAPTURES
from PIL import ImageFont

RESULTS = CAPTURES
LABELS = {"blade_shape": "刀形", "ring_broad_split": "宽分叉头双环",
          "ring_narrow_tip": "细直头双环", "tweezer_shape": "镊形"}

def font(size):
    for name in ("msyh.ttc", "NotoSansCJK-Regular.ttc", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()

from paper_fusion import apertures, contains, fuse_triplet, primary
from prop_review import review_items


# These names describe the pink demonstration props. Blade numbers have no visual labels.
TARGETS = {
    "straight_mayo_scissor": ("直剪", "ring_broad_split"),
    "hemostat": ("止血钳", "ring_narrow_tip"),
    "tissue_forceps": ("组织镊", "tweezer_shape"),
    "scalpel_handle_3_blade_10": ("3号刀柄配10号刀片", "blade_shape"),
    "scalpel_handle_3_blade_11": ("3号刀柄配11号刀片", "blade_shape"),
    "scalpel_handle_3_blade_15": ("3号刀柄配15号刀片", "blade_shape"),
    "scalpel_handle_4_blade_20": ("4号刀柄配20号刀片", "blade_shape"),
}
RINGS = {"ring_broad_split", "ring_narrow_tip"}


def _read(folder, name):
    path = (folder / name).resolve()
    if path.parent != folder or not path.is_file():
        raise ValueError("本次拍摄的图像文件缺失")
    return path


def _frame(path):
    im = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
    if im is None:
        raise ValueError("相机原图无法解码")
    return im


def _clipped(box, bounds):
    return any(abs(float(a)-b) <= 2 for a,b in zip(box, bounds))


def _broad_split_visible(image, box):
    """Confirm two adjacent enclosed finger rings and two broad distal prongs on one pink component."""
    x, y, r, b = np.rint(box).astype(int)
    crop = image[max(0, y-4):b+4, max(0, x-4):r+4]
    if min(crop.shape[:2]) < 12:
        return False
    blue, green, red = cv2.split(crop.astype(np.int16))
    chroma = np.clip(red-green + .4 * (blue-green), 0, 255).astype(np.uint8)
    cut, mask = cv2.threshold(chroma, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    if cut < 15:
        return False
    contours, tree = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if tree is None:
        return False
    body = max(range(len(contours)), key=lambda i: cv2.contourArea(contours[i]))
    circles = []
    for i, contour in enumerate(contours):
        if tree[0, i, 3] != body or len(contour) < 5:
            continue
        area = cv2.contourArea(contour)
        center, axes, _ = cv2.fitEllipse(contour)
        if (area >= 12 and min(axes) >= .40 * max(axes)
                and .72 <= 4 * area / max(1., np.pi * axes[0] * axes[1]) <= 1.20):
            circles.append((np.asarray(center), area))
    if len(circles) < 2:
        return False
    component = np.zeros_like(mask)
    cv2.drawContours(component, contours, body, 255, cv2.FILLED)
    component &= mask
    coords = np.column_stack(np.nonzero(component)[::-1])
    for (a, area_a), (b, area_b) in combinations(circles, 2):
        mid = (a + b) / 2
        span = np.linalg.norm(a-b)
        if not .45 <= area_a / area_b <= 2.2 or span < np.sqrt(max(area_a, area_b)):
            continue
        direction = coords.mean(axis=0) - mid
        direction /= max(1., np.linalg.norm(direction))
        if abs((a-b) @ direction) > .40 * span:
            continue
        normal = np.array([-direction[1], direction[0]])
        projected = (coords-mid) @ direction
        length = np.percentile(projected, 99.5)
        if length < 2.2 * span or -projected.min() > .31 * length:
            continue
        # The narrow ring prop ends in a single slender shaft. Require two
        # distinct wide runs through three consecutive sections near the tip.
        offsets = np.arange(-math.ceil(span), math.ceil(span)+1)
        count = 0
        for position in np.linspace(.84, .98, 8):
            xy = np.rint(mid + position * length * direction + offsets[:, None] * normal).astype(int)
            inside = ((xy >= 0) & (xy < component.shape[1::-1])).all(axis=1)
            row = np.zeros(len(xy), dtype=np.int8)
            row[inside] = component[xy[inside, 1], xy[inside, 0]] != 0
            runs = np.flatnonzero(np.diff(np.pad(row, 1))).reshape(-1, 2)
            runs = [pair for pair in runs if pair[1] - pair[0] >= max(2., .10 * span)]
            fork = (len(runs) == 2 and runs[1][0] - runs[0][1] >= 2
                    and runs[1][1] - runs[0][0] >= .45 * span)
            count = count + 1 if fork else 0
            if count >= 3:
                return True
    return False


def _confidence(obj, holes, bounds, head_image=None):
    """Assess a specific proposal, independently of the words the user spoke."""
    cls = obj.get("recognized_class", obj["class_name"])
    shape = (obj.get("shape_review") or {}).get("class_name")
    box = obj.get("head_box") or obj["wrist_box"]
    if _clipped(box, bounds):
        return "occluded"
    if cls in RINGS and sum(contains(box,h["xy"]) for h in holes) < 2:
        if not (cls == obj["class_name"] == "ring_broad_split" and obj["confidence"] >= .70
                and head_image is not None and _broad_split_visible(head_image, box)):
            return "occluded"
    alt = obj.get("alternative")
    if alt and obj["confidence"]-alt["confidence"] < .08 and shape != cls:
        return "uncertain"
    if shape == cls or obj["confidence"] >= .5:
        return "found"
    return "uncertain"


def _summarize(target_id, objects, wrist_hints, visible):
    label, cls = TARGETS.get(target_id, (target_id, None))
    matches = [o for o in objects if o["class_name"] == cls]
    strong = [o for o in matches if o["evidence"] == "found"]
    ambiguous = [o for o in objects if cls in o["alternatives"]]
    partial_ring = cls in RINGS and any(o["class_name"] in RINGS and o["evidence"]=="occluded" for o in objects)
    status, msg = "uncertain", "语音目标没有对应的视觉类别。"
    if cls is None:
        pass
    elif not visible:
        msg = "画面中未可靠定位工作台，请让头部镜头拍到摆放模具的区域。"
    elif cls == "blade_shape" and (matches or ambiguous or cls in wrist_hints):
        msg = "画面中有刀形候选。视觉模型尚不能核实刀柄和刀片编号。"
    elif strong:
        status, msg = "found", f"在台面可见区域找到{label}模具，外形为{LABELS[cls]}。"
        if len(strong)>1:
            msg += f"共有{len(strong)}个匹配位置。"
    elif any(o["evidence"]=="occluded" for o in matches) or partial_ring:
        status, msg = "occluded", f"画面中有局部轮廓，未看全辨别{label}所需的特征。"
    elif matches or ambiguous:
        msg = f"看到了可能的{label}，相近轮廓或检测分数不足以确认。"
    elif cls in wrist_hints:
        msg = f"腕部画面有{LABELS[cls]}候选，尚未与头部台面位置配对。"
    else:
        status, msg = "absent", f"本次台面可见识别区域内未检出{label}模具。"
    numbered = strong if status=="found" else matches+([o for o in ambiguous if o not in matches] if status=="uncertain" else [])
    return dict(id=target_id,label=label,status=status,message=msg,count=len(strong) if status=="found" else 0,
                unique=bool(status=="found" and len(strong)==1),numbers=[o["number"] for o in numbered])


def _preview(folder, image, roi, objects, results):
    x,y,r,b = map(int, roi)
    anchors = [o["anchor"] for o in objects if o["anchor"]]
    for ax,ay in anchors:
        x,y,r,b = min(x,int(ax)-12),min(y,int(ay)-12),max(r,int(ax)+12),max(b,int(ay)+12)
    x,y,r,b = max(x,0),max(y,0),min(r,image.shape[1]),min(b,image.shape[0])
    crop = Image.fromarray(cv2.cvtColor(image[y:b,x:r],cv2.COLOR_BGR2RGB))
    # Labels use display pixels even when the native workspace is small.
    scale = min(1050/crop.width, 650/crop.height)
    crop = crop.resize((round(crop.width*scale),round(crop.height*scale)),Image.Resampling.LANCZOS)
    draw = ImageDraw.Draw(crop)
    confirmed = {n for target in results if target["status"]=="found" for n in target["numbers"]}
    possible = {n for target in results if target["status"]!="found" for n in target["numbers"]}
    for obj in objects:
        color = "#0b8070" if obj["evidence"]=="found" else "#ab651f"
        if results:
            color = "#00826c" if obj["number"] in confirmed else "#ae6515" if obj["number"] in possible else "#536879"
        if obj["box"] is not None:
            a,c,e,g = obj["box"]
            px,py = (a-x)*scale,(c-y)*scale
            draw.rectangle((px,py,(e-x)*scale,(g-y)*scale),outline=color,width=3)
        else:
            ax,ay = obj["anchor"]
            px,py = (ax-x)*scale,(ay-y)*scale
            draw.ellipse((px-6,py-6,px+6,py+6),outline=color,width=3)
        left = max(0, min(crop.width-24,px))
        top = max(0,py-24)
        draw.rectangle((left,top,left+22,top+23),fill=color)
        draw.text((left+5,top),str(obj["number"]),font=font(17), fill="white")
    # Separate archived overview and query-specific overlays in case two windows
    # inspect the same saved shot.
    signature = hashlib.sha256(json.dumps(results,sort_keys=True).encode()).hexdigest()[:12] if results else ""
    dest = folder/(f"console_head_{signature}.jpg" if signature else "console_head.jpg")
    with tempfile.NamedTemporaryFile(dir=folder,suffix=".jpg",delete=False) as stream:
        staging = Path(stream.name)
    try:
        crop.save(staging,quality=92)
        staging.replace(dest)
    finally:
        staging.unlink(missing_ok=True)
    return str(dest)


def _unavailable(ids, source, reason):
    return dict(source=source,folder=None,captured_at=None,image=None,views={},objects=[],error=reason,
                results=[dict(id=k,label=TARGETS.get(k,(k,None))[0],status="unavailable",message=reason,
                              count=0,unique=False,numbers=[]) for k in ids])


def _triplet(folder):
    live = folder is None
    for attempt in range(2 if live else 1):
        if live:
            folder = capture_scene()
        folder = Path(folder).resolve()
        if folder.parent != RESULTS.resolve() or not folder.name.startswith("camera_triplet_"):
            raise ValueError("无法验证相机采集目录")
        raw = json.loads((folder/"result.json").read_bytes())
        views = {v["role"]:v for v in raw["views"]}
        head = views.get("head",{})
        if head.get("image_file") and "ranked_groups" in head:
            return folder, raw, views, head
        if not live or attempt or head.get("error") != "Camera did not deliver a frame":
            break
    error = str(head.get("error") or head.get("inference_error") or raw.get("inference_error") or "")
    if "No such file or directory" in error and "/dev/" in error:
        raise ValueError("O6 未找到头部摄像头的 USB 设备。请将头部摄像头接回原 USB 口后重新拍摄。")
    if error == "Camera did not deliver a frame":
        raise ValueError("头部摄像头本次没有返回图像，请检查接线后重拍。")
    raise ValueError(error or "本次未收到头部图像和检测结果")


def lookup(ids, *, folder=None):
    """One evaluated scene for the entire ordered request. An archive is used only explicitly."""
    source = "replay" if folder is not None else "live"
    try:
        folder, raw, views, head = _triplet(folder)
        paths = {k:str(_read(folder,v["image_file"])) for k,v in views.items() if v.get("image_file")}
        head_frame = _frame(paths["head"])
        roi = head["detection_record"]["inference_roi_xyxy"]
        if set(raw["model"]["class_names"].values()) != set(LABELS):
            raise ValueError("相机模型与当前模具类别不匹配")
        fused = fuse_triplet(folder)
        items,_ = review_items(head_frame, fused["items"])
        head_holes = apertures(head_frame, roi)
        objects, wrist_hints = [], set()
        wrist_holes = []
        right = views.get("right_wrist",{})
        if any(obj["source"]=="right_wrist" for obj in items):
            wrist_roi = right["detection_record"]["inference_roi_xyxy"]
            wrist_holes = apertures(_frame(paths["right_wrist"]),wrist_roi)
        for n,obj in enumerate(items,1):
            on_head = obj["source"] == "head"
            evidence = _confidence(obj,head_holes if on_head else wrist_holes,roi if on_head else wrist_roi,
                                   head_image=head_frame if on_head else None)
            name = obj.get("recognized_class",obj["class_name"])
            alt = obj.get("alternative")
            shape = (obj.get("shape_review") or {}).get("class_name")
            contenders = [alt["class_name"]] if alt and not shape and obj["confidence"]-alt["confidence"] < .08 else []
            objects.append(dict(number=n,label=LABELS[name],class_name=name,box=obj["head_box"],
                                anchor=obj["head_anchor"],evidence=evidence,alternatives=contenders))
        for role in ("right_wrist","left_wrist"):
            v = views.get(role,{})
            if "ranked_groups" in v:
                wr_roi = v["detection_record"]["inference_roi_xyxy"]
                wrist_hints.update(p["class_name"] for p in primary(v)
                                   if p["confidence"] >= .5 and not _clipped(p["box"],wr_roi))
        # A wrist proposal outside the head workspace cannot establish a table position.
        visible = head.get("table_region",{}).get("verified") or head.get("roi_detail",{}).get("light_surface_components",0)>0
        results = [_summarize(i,objects,wrist_hints,visible) for i in ids]
        picture = _preview(folder,head_frame,roi,objects,results)
        exposure = {}
        for role, view in views.items():
            ae = view.get("auto_exposure")
            if not ae:
                continue
            status = ae["status"]
            level = (ae.get("after") or {}).get("level")
            message = {"balanced": "测光合适，保留原设置", "adjusted": "已自动调整并复测",
                       "limited": {"bright": "已测光，工作区域仍偏亮", "dark": "已测光，工作区域仍偏暗"}.get(level,"已测光，保留较合适的设置"),
                       "unavailable": "本次未能验证自动曝光"}.get(status, "曝光状态未知")
            exposure[role] = {"status": status, "message": message, "value": ae.get("value")}
        return dict(source=source,folder=str(folder),captured_at=raw["utc"],image=picture,views=paths,
                    head_camera={"device": head.get("device_by_path"), "size": list(head_frame.shape[1::-1])},
                    results=results,objects=objects,error=None,exposure=exposure)
    except (OSError, RuntimeError, ValueError, KeyError, cv2.error) as exc:
        return _unavailable(ids,source,str(exc))


def last_snapshot():
    p = max((d for d in RESULTS.glob("camera_triplet_*") if (d/"result.json").is_file()),default=None)
    if p is None:
        return None
    result = lookup([], folder=p)
    result["source"] = "archive"
    return result
