# rescue 分支（be39ffe）vs main：A8 MATRIX 差异记录（2026-09-09）

## 目的

rescue 分支（`be39ffe`，A8 v_min matrix 旧口径）内容已被归档 tag
`archive-rescue-be39ffe`（annotated，2026-09-09 创建，peel = `be39ffe`）完整覆盖。
本文档在删除该分支前，固化「rescue 与 main 的矩阵差异」，供审计/回溯使用；
分支删除后，证据仍由 archive tag 与本文档共同承载。

## 引用状态（删除前核验）

| 引用 | 指向 |
|---|---|
| rescue 分支（origin，归档于 `archive-rescue-be39ffe`） | `be39ffe` |
| `archive-rescue-be39ffe^{commit}` | `be39ffe`（同点，merge-base --is-ancestor 通过） |
| `main` / `v0.3.0-engineered^{commit}` | `d920f4c` |
| 相对进度 | main vs rescue = 28 ahead / 1 behind（rescue 唯一独有提交即 be39ffe 本身） |

## 差异摘要（main..rescue，`results/a8_replay/MATRIX.md`）

| 维度 | rescue（be39ffe） | main（d920f4c） |
|---|---|---|
| A8.0 alignment | accepted_arc_final = **0.631**（旧口径） | **0.988**（封板口径） |
| backward 4 格 | 全 STALL（progress 0.1376–0.1521，qp_failures 78/26/4） | **COMPLETED** progress 0.9958、qp_failures 0（postseal2 rerun：reverse_link 退役改半圆帽后重跑） |
| cap / nocap 复现对照 | 仅旧版复制行 | 封板表 + postseal2 rerun 段：cap 逐字节复现、nocap/backward 由几何语义修正驱动 |
| 覆盖说明 | 无 postseal2 说明段 | 注明 "Sealed table above is kept as the historical record" |
| geo/*.json | 旧版 | a8_cap/a8_nocap/a8_cap_real 逐字节一致，仅 a8_backward.json 随语义修正变化 |

## rescue 独有内容的处置依据

- rescue 相对 main 的 226 文件差异中，独有新增主要为**已入库构建垃圾**
  （`build_core/`、`log/`），非证据性内容；
- 证据性内容（12 格矩阵 + 3 复制行 + 工具）在 main 上以更新口径存在
  （`results/a8_replay/` 封板表 + postseal2 段 + cell_*.json）；
- `be39ffe` 提交本身已被 `archive-rescue-be39ffe` tag 完整引用，删除分支零丢失。

## 处置结论

1. `archive-rescue-be39ffe` tag 永久保留（不动）；
2. rescue 分支按只读审计 C 类结论删除（普通分支删除，在本记录之后执行）；
3. 本文档入库 `results/a8_replay/rescue_vs_main_matrix_diff.md`，作为删除前快照说明。
