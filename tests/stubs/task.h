#pragma once
/* Host execution is single-threaded. IRQ masking is checked in source review. */
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
