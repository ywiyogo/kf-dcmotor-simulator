#include "PastelTheme.hpp"
#include <imgui.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <cmath>

namespace kf::ui::themes {

PastelTheme::PastelTheme(Variant variant) 
    : variant_(variant), smooth_transitions_(true) {
    initialize_colors();
}

void PastelTheme::apply() {
    apply_colors();
    apply_styling();
}

void PastelTheme::apply_with_config(const ThemeConfig& config) {
    config_ = config;
    apply();
}

void PastelTheme::set_variant(Variant variant) {
    variant_ = variant;
    initialize_colors();
    color_cache_.clear(); // Clear cache when variant changes
}

ImVec4 PastelTheme::get_plot_color(std::size_t index) {
    return PastelColors::PLOT_COLORS[index % PastelColors::PLOT_COLORS.size()];
}

ImVec4 PastelTheme::get_status_color(const std::string& status) {
    if (status == "success") return PastelColors::SUCCESS;
    if (status == "warning") return PastelColors::WARNING;
    if (status == "error") return PastelColors::ERROR;
    if (status == "info") return PastelColors::INFO;
    return PastelColors::TEXT_PRIMARY;
}

ImVec4 PastelTheme::create_hover_color(const ImVec4& base_color, float hover_factor) {
    auto hsv = ThemeUtils::rgb_to_hsv(base_color);
    hsv[2] = std::min(1.0f, hsv[2] + hover_factor); // Increase brightness
    return ThemeUtils::hsv_to_rgb(hsv);
}

ImVec4 PastelTheme::create_active_color(const ImVec4& base_color, float active_factor) {
    auto hsv = ThemeUtils::rgb_to_hsv(base_color);
    hsv[1] = std::min(1.0f, hsv[1] + active_factor * 0.5f); // Increase saturation
    hsv[2] = std::max(0.0f, hsv[2] - active_factor * 0.3f); // Decrease brightness slightly
    return ThemeUtils::hsv_to_rgb(hsv);
}

ImVec4 PastelTheme::lerp_color(const ImVec4& color1, const ImVec4& color2, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return ImVec4{
        color1.x + t * (color2.x - color1.x),
        color1.y + t * (color2.y - color1.y),
        color1.z + t * (color2.z - color1.z),
        color1.w + t * (color2.w - color1.w)
    };
}

ImVec4 PastelTheme::apply_gradient(const ImVec4& base_color, float gradient_factor) {
    auto hsv = ThemeUtils::rgb_to_hsv(base_color);
    hsv[2] = std::min(1.0f, hsv[2] + gradient_factor);
    return ThemeUtils::hsv_to_rgb(hsv);
}

void PastelTheme::set_color_override(ImGuiCol color_name, const ImVec4& color) {
    color_overrides_[color_name] = color;
}

void PastelTheme::remove_color_override(ImGuiCol color_name) {
    color_overrides_.erase(color_name);
}

void PastelTheme::clear_overrides() {
    color_overrides_.clear();
}

bool PastelTheme::save_to_file(const std::string& filename) const {
    try {
        std::ofstream file(filename);
        if (!file.is_open()) return false;

        file << "# Pastel Theme Configuration\n";
        file << "variant=" << static_cast<int>(variant_) << "\n";
        file << "smooth_transitions=" << (smooth_transitions_ ? "1" : "0") << "\n";
        
        // Save config
        file << "window_rounding=" << config_.window_rounding << "\n";
        file << "child_rounding=" << config_.child_rounding << "\n";
        file << "frame_rounding=" << config_.frame_rounding << "\n";
        file << "popup_rounding=" << config_.popup_rounding << "\n";
        file << "scrollbar_rounding=" << config_.scrollbar_rounding << "\n";
        file << "grab_rounding=" << config_.grab_rounding << "\n";
        file << "tab_rounding=" << config_.tab_rounding << "\n";
        
        return true;
    } catch (...) {
        return false;
    }
}

bool PastelTheme::load_from_file(const std::string& filename) {
    try {
        std::ifstream file(filename);
        if (!file.is_open()) return false;

        std::string line;
        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') continue;
            
            auto eq_pos = line.find('=');
            if (eq_pos == std::string::npos) continue;
            
            std::string key = line.substr(0, eq_pos);
            std::string value = line.substr(eq_pos + 1);
            
            if (key == "variant") {
                variant_ = static_cast<Variant>(std::stoi(value));
            } else if (key == "smooth_transitions") {
                smooth_transitions_ = (value == "1");
            } else if (key == "window_rounding") {
                config_.window_rounding = std::stof(value);
            } else if (key == "child_rounding") {
                config_.child_rounding = std::stof(value);
            } else if (key == "frame_rounding") {
                config_.frame_rounding = std::stof(value);
            } else if (key == "popup_rounding") {
                config_.popup_rounding = std::stof(value);
            } else if (key == "scrollbar_rounding") {
                config_.scrollbar_rounding = std::stof(value);
            } else if (key == "grab_rounding") {
                config_.grab_rounding = std::stof(value);
            } else if (key == "tab_rounding") {
                config_.tab_rounding = std::stof(value);
            }
        }
        
        initialize_colors();
        return true;
    } catch (...) {
        return false;
    }
}

std::string PastelTheme::get_theme_name() const {
    switch (variant_) {
        case Variant::LIGHT: return "Pastel Light";
        case Variant::SOFT: return "Pastel Soft";
        case Variant::VIBRANT: return "Pastel Vibrant";
        case Variant::CONTRAST: return "Pastel Contrast";
        default: return "Pastel Theme";
    }
}

void PastelTheme::initialize_colors() {
    color_cache_.clear();
}

void PastelTheme::apply_colors() {
    ImGuiStyle& style = ImGui::GetStyle();
    
    // Text colors
    style.Colors[ImGuiCol_Text] = adjust_color_for_variant(PastelColors::TEXT_PRIMARY);
    style.Colors[ImGuiCol_TextDisabled] = adjust_color_for_variant(PastelColors::TEXT_DISABLED);
    style.Colors[ImGuiCol_TextSelectedBg] = adjust_color_for_variant(PastelColors::PRIMARY_LIGHT);
    
    // Window colors
    style.Colors[ImGuiCol_WindowBg] = adjust_color_for_variant(PastelColors::BACKGROUND);
    style.Colors[ImGuiCol_ChildBg] = adjust_color_for_variant(PastelColors::SURFACE);
    style.Colors[ImGuiCol_PopupBg] = adjust_color_for_variant(PastelColors::SURFACE);
    
    // Border colors
    style.Colors[ImGuiCol_Border] = adjust_color_for_variant(PastelColors::OUTLINE);
    style.Colors[ImGuiCol_BorderShadow] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    
    // Frame colors
    style.Colors[ImGuiCol_FrameBg] = adjust_color_for_variant(PastelColors::SURFACE_VARIANT);
    style.Colors[ImGuiCol_FrameBgHovered] = create_hover_color(style.Colors[ImGuiCol_FrameBg]);
    style.Colors[ImGuiCol_FrameBgActive] = create_active_color(style.Colors[ImGuiCol_FrameBg]);
    
    // Title bar colors
    style.Colors[ImGuiCol_TitleBg] = adjust_color_for_variant(PastelColors::PRIMARY_LIGHT);
    style.Colors[ImGuiCol_TitleBgActive] = adjust_color_for_variant(PastelColors::PRIMARY);
    style.Colors[ImGuiCol_TitleBgCollapsed] = style.Colors[ImGuiCol_TitleBg];
    
    // Menu bar colors
    style.Colors[ImGuiCol_MenuBarBg] = adjust_color_for_variant(PastelColors::SURFACE);
    
    // Scrollbar colors
    style.Colors[ImGuiCol_ScrollbarBg] = adjust_color_for_variant(PastelColors::BACKGROUND);
    style.Colors[ImGuiCol_ScrollbarGrab] = adjust_color_for_variant(PastelColors::OUTLINE);
    style.Colors[ImGuiCol_ScrollbarGrabHovered] = create_hover_color(style.Colors[ImGuiCol_ScrollbarGrab]);
    style.Colors[ImGuiCol_ScrollbarGrabActive] = create_active_color(style.Colors[ImGuiCol_ScrollbarGrab]);
    
    // Check mark colors
    style.Colors[ImGuiCol_CheckMark] = adjust_color_for_variant(PastelColors::TERTIARY_ACCENT);
    
    // Slider colors
    style.Colors[ImGuiCol_SliderGrab] = adjust_color_for_variant(PastelColors::PRIMARY_ACCENT);
    style.Colors[ImGuiCol_SliderGrabActive] = create_active_color(style.Colors[ImGuiCol_SliderGrab]);
    
    // Button colors
    style.Colors[ImGuiCol_Button] = adjust_color_for_variant(PastelColors::PRIMARY);
    style.Colors[ImGuiCol_ButtonHovered] = create_hover_color(style.Colors[ImGuiCol_Button]);
    style.Colors[ImGuiCol_ButtonActive] = create_active_color(style.Colors[ImGuiCol_Button]);
    
    // Header colors
    style.Colors[ImGuiCol_Header] = adjust_color_for_variant(PastelColors::PRIMARY_LIGHT);
    style.Colors[ImGuiCol_HeaderHovered] = create_hover_color(style.Colors[ImGuiCol_Header]);
    style.Colors[ImGuiCol_HeaderActive] = create_active_color(style.Colors[ImGuiCol_Header]);
    
    // Separator colors
    style.Colors[ImGuiCol_Separator] = adjust_color_for_variant(PastelColors::OUTLINE_VARIANT);
    style.Colors[ImGuiCol_SeparatorHovered] = create_hover_color(style.Colors[ImGuiCol_Separator]);
    style.Colors[ImGuiCol_SeparatorActive] = create_active_color(style.Colors[ImGuiCol_Separator]);
    
    // Resize grip colors
    style.Colors[ImGuiCol_ResizeGrip] = adjust_color_for_variant(PastelColors::PRIMARY);
    style.Colors[ImGuiCol_ResizeGripHovered] = create_hover_color(style.Colors[ImGuiCol_ResizeGrip]);
    style.Colors[ImGuiCol_ResizeGripActive] = create_active_color(style.Colors[ImGuiCol_ResizeGrip]);
    
    // Tab colors
    style.Colors[ImGuiCol_Tab] = adjust_color_for_variant(PastelColors::SURFACE_VARIANT);
    style.Colors[ImGuiCol_TabHovered] = create_hover_color(style.Colors[ImGuiCol_Tab]);
    style.Colors[ImGuiCol_TabSelected] = adjust_color_for_variant(PastelColors::PRIMARY_LIGHT);
    style.Colors[ImGuiCol_TabDimmed] = style.Colors[ImGuiCol_Tab];
    style.Colors[ImGuiCol_TabDimmedSelected] = lerp_color(style.Colors[ImGuiCol_Tab], style.Colors[ImGuiCol_TabSelected], 0.5f);
    
    // Docking colors
    style.Colors[ImGuiCol_DockingPreview] = adjust_color_for_variant(PastelColors::PRIMARY);
    style.Colors[ImGuiCol_DockingEmptyBg] = adjust_color_for_variant(PastelColors::BACKGROUND);
    
    // Plot colors
    style.Colors[ImGuiCol_PlotLines] = adjust_color_for_variant(PastelColors::PLOT_COLORS[0]);
    style.Colors[ImGuiCol_PlotLinesHovered] = create_hover_color(style.Colors[ImGuiCol_PlotLines]);
    style.Colors[ImGuiCol_PlotHistogram] = adjust_color_for_variant(PastelColors::PLOT_COLORS[1]);
    style.Colors[ImGuiCol_PlotHistogramHovered] = create_hover_color(style.Colors[ImGuiCol_PlotHistogram]);
    
    // Table colors
    style.Colors[ImGuiCol_TableHeaderBg] = adjust_color_for_variant(PastelColors::SURFACE_VARIANT);
    style.Colors[ImGuiCol_TableBorderStrong] = adjust_color_for_variant(PastelColors::OUTLINE);
    style.Colors[ImGuiCol_TableBorderLight] = adjust_color_for_variant(PastelColors::OUTLINE_VARIANT);
    style.Colors[ImGuiCol_TableRowBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);
    style.Colors[ImGuiCol_TableRowBgAlt] = adjust_color_for_variant(PastelColors::SURFACE);
    
    // Apply color overrides
    for (const auto& [color_id, color] : color_overrides_) {
        style.Colors[color_id] = color;
    }
}

void PastelTheme::apply_styling() {
    ImGuiStyle& style = ImGui::GetStyle();
    
    // Window styling
    style.WindowRounding = config_.window_rounding;
    style.ChildRounding = config_.child_rounding;
    style.FrameRounding = config_.frame_rounding;
    style.PopupRounding = config_.popup_rounding;
    style.ScrollbarRounding = config_.scrollbar_rounding;
    style.GrabRounding = config_.grab_rounding;
    style.TabRounding = config_.tab_rounding;
    
    // Border sizes
    style.WindowBorderSize = config_.window_border_size;
    style.ChildBorderSize = config_.child_border_size;
    style.PopupBorderSize = config_.popup_border_size;
    style.FrameBorderSize = config_.frame_border_size;
    style.TabBorderSize = config_.tab_border_size;
    
    // Padding and spacing
    style.WindowPadding = config_.window_padding;
    style.FramePadding = config_.frame_padding;
    style.CellPadding = config_.cell_padding;
    style.ItemSpacing = config_.item_spacing;
    style.ItemInnerSpacing = config_.item_inner_spacing;
    style.ButtonTextAlign = config_.button_text_align;
    style.SelectableTextAlign = config_.selectable_text_align;
    
    // Sizes
    style.IndentSpacing = config_.indent_spacing;
    style.ColumnsMinSpacing = config_.columns_min_spacing;
    style.ScrollbarSize = config_.scrollbar_size;
    style.GrabMinSize = config_.grab_min_size;
    
    // Alpha values
    style.Alpha = config_.alpha;
    style.DisabledAlpha = config_.disabled_alpha;
}

ImVec4 PastelTheme::adjust_color_for_variant(const ImVec4& base_color) const {
    switch (variant_) {
        case Variant::LIGHT:
            return base_color;
            
        case Variant::SOFT:
            return ThemeUtils::adjust_saturation(base_color, 0.8f);
            
        case Variant::VIBRANT:
            return ThemeUtils::adjust_saturation(base_color, 1.2f);
            
        case Variant::CONTRAST:
            return ThemeUtils::adjust_brightness(base_color, 1.1f);
            
        default:
            return base_color;
    }
}

// Theme utility functions implementation
namespace ThemeUtils {

std::array<float, 4> rgb_to_hsv(const ImVec4& color) {
    float r = color.x, g = color.y, b = color.z, a = color.w;
    
    float max_val = std::max({r, g, b});
    float min_val = std::min({r, g, b});
    float delta = max_val - min_val;
    
    float h = 0.0f, s = 0.0f, v = max_val;
    
    if (delta > 0.0001f) {
        s = delta / max_val;
        
        if (max_val == r) {
            h = (g - b) / delta;
            if (h < 0.0f) h += 6.0f;
        } else if (max_val == g) {
            h = 2.0f + (b - r) / delta;
        } else {
            h = 4.0f + (r - g) / delta;
        }
        h /= 6.0f;
    }
    
    return {h, s, v, a};
}

ImVec4 hsv_to_rgb(const std::array<float, 4>& hsv) {
    float h = hsv[0] * 6.0f;
    float s = hsv[1];
    float v = hsv[2];
    float a = hsv[3];
    
    int i = static_cast<int>(std::floor(h));
    float f = h - i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));
    
    switch (i % 6) {
        case 0: return ImVec4(v, t, p, a);
        case 1: return ImVec4(q, v, p, a);
        case 2: return ImVec4(p, v, t, a);
        case 3: return ImVec4(p, q, v, a);
        case 4: return ImVec4(t, p, v, a);
        case 5: return ImVec4(v, p, q, a);
        default: return ImVec4(v, v, v, a);
    }
}

ImVec4 adjust_saturation(const ImVec4& color, float saturation_factor) {
    auto hsv = rgb_to_hsv(color);
    hsv[1] = std::clamp(hsv[1] * saturation_factor, 0.0f, 1.0f);
    return hsv_to_rgb(hsv);
}

ImVec4 adjust_brightness(const ImVec4& color, float brightness_factor) {
    auto hsv = rgb_to_hsv(color);
    hsv[2] = std::clamp(hsv[2] * brightness_factor, 0.0f, 1.0f);
    return hsv_to_rgb(hsv);
}

ImVec4 get_complementary_color(const ImVec4& color) {
    auto hsv = rgb_to_hsv(color);
    hsv[0] = std::fmod(hsv[0] + 0.5f, 1.0f);
    return hsv_to_rgb(hsv);
}

std::vector<ImVec4> generate_palette(const ImVec4& base_color, std::size_t count) {
    std::vector<ImVec4> palette;
    palette.reserve(count);
    
    auto base_hsv = rgb_to_hsv(base_color);
    
    for (std::size_t i = 0; i < count; ++i) {
        auto hsv = base_hsv;
        hsv[0] = std::fmod(hsv[0] + static_cast<float>(i) / count, 1.0f);
        palette.push_back(hsv_to_rgb(hsv));
    }
    
    return palette;
}

bool is_dark_color(const ImVec4& color) {
    // Calculate luminance using standard weights
    float luminance = 0.299f * color.x + 0.587f * color.y + 0.114f * color.z;
    return luminance < 0.5f;
}

ImVec4 get_text_color_for_background(const ImVec4& background_color) {
    return is_dark_color(background_color) ? 
           PastelColors::TEXT_ON_PRIMARY : 
           PastelColors::TEXT_PRIMARY;
}

} // namespace ThemeUtils

} // namespace kf::ui::themes