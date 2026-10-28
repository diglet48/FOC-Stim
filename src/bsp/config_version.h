#ifndef FOCSTIM_CONFIG_VERSION_H
#define FOCSTIM_CONFIG_VERSION_H

constexpr int focstim_api_version_major = 2;
constexpr int focstim_api_version_minor = 0;
constexpr int focstim_api_version_revision = 3;
static constexpr char focstim_api_branch_name[] = "main";   // mainline restim requires this to be "main".
// can be whatever. Used to identify the firmware variant.
#if defined(TRANSFORMER_42TL004)
static constexpr char focstim_api_comment[] = "42TL004";
#elif defined(TRANSFORMER_42TL001)
static constexpr char focstim_api_comment[] = "42TL001";
#else
static constexpr char focstim_api_comment[] = "";
#endif

#endif