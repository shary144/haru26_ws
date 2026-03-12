template<class Derived>
class ParamMixin {
public:
    template<typename T>
    void param(const std::string& name, T& variable) {
        // declare_parameter でデフォルト値を公開
        this->declare_parameter<T>(name, variable);

        // YAML/CLI の値があれば反映
        variable = this->get_parameter(name).get_value<T>();

        // 動的変更にも対応
        callbacks_.push_back(
            this->add_on_set_parameters_callback(
                [&, name](const std::vector<rclcpp::Parameter>& params) {
                    for (auto & p : params) {
                        if (p.get_name() == name) {
                            variable = p.get_value<T>();
                        }
                    }
                    rcl_interfaces::msg::SetParametersResult result;
                    result.successful = true;
                    return result;
                }
            )
        );
    }

private:
    std::vector<rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr> callbacks_;
};