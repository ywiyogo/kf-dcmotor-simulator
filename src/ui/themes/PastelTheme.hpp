#pragma once

#include <imgui.h>
#include <array>
#include <string>
#include <unordered_map>
#include <functional>

namespace kf::ui::themes {

/**
 * @brief Modern pastel color palette for the Kalman Filter application
 * 
 * Provides a beautiful, eye-friendly color scheme with soft pastel colors
 * that enhance readability and provide a professional appearance.
 */
struct PastelColors {
    // Primary palette - soft blues and teals
    static constexpr ImVec4 PRIMARY_LIGHT = ImVec4{0.85f, 0.92f, 0.98f, 1.0f};    // Very light blue
    static constexpr ImVec4 PRIMARY = ImVec4{0.70f, 0.85f, 0.95f, 1.0f};          // Soft blue
    static constexpr ImVec4 PRIMARY_DARK = ImVec4{0.55f, 0.75f, 0.90f, 1.0f};     // Medium blue
    static constexpr ImVec4 PRIMARY_ACCENT = ImVec4{0.40f, 0.65f, 0.85f, 1.0f};   // Accent blue
    
    // Secondary palette - warm peach and coral
    static constexpr ImVec4 SECONDARY_LIGHT = ImVec4{0.98f, 0.92f, 0.85f, 1.0f};  // Light peach
    static constexpr ImVec4 SECONDARY = ImVec4{0.95f, 0.85f, 0.70f, 1.0f};        // Soft peach
    static constexpr ImVec4 SECONDARY_DARK = ImVec4{0.90f, 0.75f, 0.55f, 1.0f};   // Medium peach
    static constexpr ImVec4 SECONDARY_ACCENT = ImVec4{0.85f, 0.65f, 0.40f, 1.0f}; // Accent coral
    
    // Tertiary palette - soft greens
    static constexpr ImVec4 TERTIARY_LIGHT = ImVec4{0.90f, 0.98f, 0.85f, 1.0f};   // Light mint
    static constexpr ImVec4 TERTIARY = ImVec4{0.80f, 0.95f, 0.70f, 1.0f};         // Soft green
    static constexpr ImVec4 TERTIARY_DARK = ImVec4{0.70f, 0.90f, 0.55f, 1.0f};    // Medium green
    static constexpr ImVec4 TERTIARY_ACCENT = ImVec4{0.60f, 0.85f, 0.40f, 1.0f};  // Accent green
    
    // Neutral palette - warm grays
    static constexpr ImVec4 BACKGROUND = ImVec4{0.97f, 0.97f, 0.98f, 1.0f};       // Very light gray
    static constexpr ImVec4 SURFACE = ImVec4{0.94f, 0.94f, 0.96f, 1.0f};          // Light gray surface
    static constexpr ImVec4 SURFACE_VARIANT = ImVec4{0.91f, 0.91f, 0.93f, 1.0f};  // Medium gray
    static constexpr ImVec4 OUTLINE = ImVec4{0.75f, 0.75f, 0.80f, 1.0f};          // Gray outline
    static constexpr ImVec4 OUTLINE_VARIANT = ImVec4{0.85f, 0.85f, 0.88f, 1.0f};  // Light outline
    
    // Text colors
    static constexpr ImVec4 TEXT_PRIMARY = ImVec4{0.15f, 0.15f, 0.20f, 1.0f};      // Dark text
    static constexpr ImVec4 TEXT_SECONDARY = ImVec4{0.35f, 0.35f, 0.40f, 1.0f};    // Medium text
    static constexpr ImVec4 TEXT_DISABLED = ImVec4{0.55f, 0.55f, 0.60f, 1.0f};     // Disabled text
    static constexpr ImVec4 TEXT_ON_PRIMARY = ImVec4{0.95f, 0.95f, 0.98f, 1.0f};   // Text on primary
    
    // Status colors - soft versions
    static constexpr ImVec4 SUCCESS = ImVec4{0.70f, 0.90f, 0.60f, 1.0f};          // Soft green
    static constexpr ImVec4 WARNING = ImVec4{0.95f, 0.85f, 0.50f, 1.0f};          // Soft yellow
    static constexpr ImVec4 ERROR = ImVec4{0.90f, 0.65f, 0.60f, 1.0f};            // Soft red
    static constexpr ImVec4 INFO = ImVec4{0.65f, 0.80f, 0.95f, 1.0f};             // Soft blue
    
    // Plot colors - vibrant but harmonious
    static constexpr std::array<ImVec4, 8> PLOT_COLORS = {{
        ImVec4{0.40f, 0.65f, 0.85f, 1.0f}, // Blue
        ImVec4{0.85f, 0.65f, 0.40f, 1.0f}, // Orange
        ImVec4{0.60f, 0.85f, 0.40f, 1.0f}, // Green
        ImVec4{0.85f, 0.40f, 0.65f, 1.0f}, // Pink
        ImVec4{0.65f, 0.40f, 0.85f, 1.0f}, // Purple
        ImVec4{0.40f, 0.85f, 0.65f, 1.0f}, // Teal
        ImVec4{0.85f, 0.85f, 0.40f, 1.0f}, // Yellow
        ImVec4{0.85f, 0.50f, 0.40f, 1.0f}  // Red
    }};
};

/**
 * @brief Comprehensive theme configuration for the application
 */
struct ThemeConfig {
    // Window styling
    float window_rounding = 8.0f;
    float child_rounding = 6.0f;
    float frame_rounding = 4.0f;
    float popup_rounding = 6.0f;
    float scrollbar_rounding = 4.0f;
    float grab_rounding = 4.0f;
    float tab_rounding = 4.0f;
    
    // Border and spacing
    float window_border_size = 1.0f;
    float child_border_size = 1.0f;
    float popup_border_size = 1.0f;
    float frame_border_size = 0.0f;
    float tab_border_size = 0.0f;
    
    // Padding and spacing
    ImVec2 window_padding = ImVec2{12.0f, 12.0f};
    ImVec2 frame_padding = ImVec2{8.0f, 6.0f};
    ImVec2 cell_padding = ImVec2{6.0f, 4.0f};
    ImVec2 item_spacing = ImVec2{8.0f, 6.0f};
    ImVec2 item_inner_spacing = ImVec2{6.0f, 4.0f};
    ImVec2 button_text_align = ImVec2{0.5f, 0.5f};
    ImVec2 selectable_text_align = ImVec2{0.0f, 0.5f};
    
    // Sizes
    float indent_spacing = 20.0f;
    float columns_min_spacing = 8.0f;
    float scrollbar_size = 14.0f;
    float grab_min_size = 12.0f;
    
    // Alpha values
    float alpha = 1.0f;
    float disabled_alpha = 0.6f;
    
    // Animation settings
    bool enable_animations = true;
    float animation_speed = 1.0f;
    float hover_transition_time = 0.15f;
    float active_transition_time = 0.08f;
};

/**
 * @brief Advanced pastel theme manager with animation support
 */
class PastelTheme {
public:
    enum class Variant {
        LIGHT,      // Standard light theme
        SOFT,       // Even softer colors
        VIBRANT,    // Slightly more saturated
        CONTRAST    // Higher contrast version
    };

    /**
     * @brief Constructor with theme variant
     * @param variant Theme variant to use
     */
    explicit PastelTheme(Variant variant = Variant::LIGHT);

    /**
     * @brief Apply the pastel theme to ImGui
     */
    void apply();

    /**
     * @brief Apply theme with custom configuration
     * @param config Custom theme configuration
     */
    void apply_with_config(const ThemeConfig& config);

    /**
     * @brief Get current theme configuration
     * @return Current theme config
     */
    const ThemeConfig& get_config() const { return config_; }

    /**
     * @brief Set theme configuration
     * @param config New theme configuration
     */
    void set_config(const ThemeConfig& config) { config_ = config; }

    /**
     * @brief Get theme variant
     * @return Current variant
     */
    Variant get_variant() const { return variant_; }

    /**
     * @brief Set theme variant
     * @param variant New variant
     */
    void set_variant(Variant variant);

    /**
     * @brief Get plot color by index
     * @param index Color index (0-7)
     * @return Plot color
     */
    static ImVec4 get_plot_color(std::size_t index);

    /**
     * @brief Get status color
     * @param status Status type ("success", "warning", "error", "info")
     * @return Status color
     */
    static ImVec4 get_status_color(const std::string& status);

    /**
     * @brief Create hover effect color
     * @param base_color Base color
     * @param hover_factor Hover intensity (0.0-1.0)
     * @return Hover color
     */
    static ImVec4 create_hover_color(const ImVec4& base_color, float hover_factor = 0.1f);

    /**
     * @brief Create active/pressed effect color
     * @param base_color Base color
     * @param active_factor Active intensity (0.0-1.0)
     * @return Active color
     */
    static ImVec4 create_active_color(const ImVec4& base_color, float active_factor = 0.2f);

    /**
     * @brief Interpolate between two colors
     * @param color1 First color
     * @param color2 Second color
     * @param t Interpolation factor (0.0-1.0)
     * @return Interpolated color
     */
    static ImVec4 lerp_color(const ImVec4& color1, const ImVec4& color2, float t);

    /**
     * @brief Apply subtle gradient to color
     * @param base_color Base color
     * @param gradient_factor Gradient intensity
     * @return Gradient color
     */
    static ImVec4 apply_gradient(const ImVec4& base_color, float gradient_factor = 0.05f);

    /**
     * @brief Set custom color override
     * @param color_name ImGui color name
     * @param color Custom color
     */
    void set_color_override(ImGuiCol color_name, const ImVec4& color);

    /**
     * @brief Remove color override
     * @param color_name ImGui color name
     */
    void remove_color_override(ImGuiCol color_name);

    /**
     * @brief Clear all color overrides
     */
    void clear_overrides();

    /**
     * @brief Save theme to file
     * @param filename Output filename
     * @return true if successful
     */
    bool save_to_file(const std::string& filename) const;

    /**
     * @brief Load theme from file
     * @param filename Input filename
     * @return true if successful
     */
    bool load_from_file(const std::string& filename);

    /**
     * @brief Get theme name
     * @return Theme name string
     */
    std::string get_theme_name() const;

    /**
     * @brief Enable/disable smooth transitions
     * @param enable Enable smooth transitions
     */
    void enable_smooth_transitions(bool enable) { smooth_transitions_ = enable; }

    /**
     * @brief Check if smooth transitions are enabled
     * @return true if enabled
     */
    bool has_smooth_transitions() const { return smooth_transitions_; }

private:
    /**
     * @brief Initialize color palette based on variant
     */
    void initialize_colors();

    /**
     * @brief Apply colors to ImGui style
     */
    void apply_colors();

    /**
     * @brief Apply styling parameters
     */
    void apply_styling();

    /**
     * @brief Adjust colors for variant
     * @param base_color Base color to adjust
     * @return Adjusted color
     */
    ImVec4 adjust_color_for_variant(const ImVec4& base_color) const;

    Variant variant_;
    ThemeConfig config_;
    std::unordered_map<ImGuiCol, ImVec4> color_overrides_;
    bool smooth_transitions_;
    
    // Color cache for performance
    mutable std::unordered_map<std::string, ImVec4> color_cache_;
};

/**
 * @brief Utility functions for theme management
 */
namespace ThemeUtils {
    /**
     * @brief Convert ImVec4 color to HSV
     * @param color RGB color
     * @return HSV values {h, s, v, a}
     */
    std::array<float, 4> rgb_to_hsv(const ImVec4& color);

    /**
     * @brief Convert HSV color to ImVec4
     * @param hsv HSV values {h, s, v, a}
     * @return RGB color
     */
    ImVec4 hsv_to_rgb(const std::array<float, 4>& hsv);

    /**
     * @brief Adjust color saturation
     * @param color Input color
     * @param saturation_factor Saturation multiplier
     * @return Adjusted color
     */
    ImVec4 adjust_saturation(const ImVec4& color, float saturation_factor);

    /**
     * @brief Adjust color brightness
     * @param color Input color
     * @param brightness_factor Brightness multiplier
     * @return Adjusted color
     */
    ImVec4 adjust_brightness(const ImVec4& color, float brightness_factor);

    /**
     * @brief Get complementary color
     * @param color Input color
     * @return Complementary color
     */
    ImVec4 get_complementary_color(const ImVec4& color);

    /**
     * @brief Generate color palette
     * @param base_color Base color
     * @param count Number of colors to generate
     * @return Vector of harmonious colors
     */
    std::vector<ImVec4> generate_palette(const ImVec4& base_color, std::size_t count);

    /**
     * @brief Check if color is dark
     * @param color Color to check
     * @return true if color is considered dark
     */
    bool is_dark_color(const ImVec4& color);

    /**
     * @brief Get appropriate text color for background
     * @param background_color Background color
     * @return Text color (light or dark)
     */
    ImVec4 get_text_color_for_background(const ImVec4& background_color);
}

} // namespace kf::ui::themes