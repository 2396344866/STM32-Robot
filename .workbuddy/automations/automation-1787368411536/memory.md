# 自动化执行记录：四足机器人固件 github 逐行同步

## 机制
- 脚本：`github_sync.sh`，按 `更新github.txt` 每行作为一次独立 commit，每天推进一行。
- 状态文件：`.github_sync_state` 记录已完成行数（committed_count）。
- 进度日志：`.github_sync_log`。
- 不自动 push（push 由用户本地执行）；脚本运行无 `--push` 参数。

## 执行历史
- 2026-08-22 11:13：LINE 1 COMMIT（修正 NVIC 优先级违规 / hal_tim_ic.c）→ 状态 1/25
- 2026-08-23 08:53：LINE 2 COMMIT（I2C 总线恢复 / hal_hard_i2c.c）→ 状态 2/25

## 当前进度
- committed_count = 2 / 25（截至 2026-08-23）
- 下次运行将处理第 3 行：ADC/DMA FIFO 半满中断 + 双缓冲（映射文件 hal_adc_dma.c / hal_adc_dma.h）

## 备注
- 运行偶有 CRLF→LF 警告（git autocrlf），不影响提交。
- 全部 25 行完成后脚本输出 [DONE]，届时提醒用户本地 `git push`。
