module.exports = function (context, options) {
  return {
    name: "custom-webpack-proxy-plugin",
    configureWebpack(config, isServer, utils) {
      return {
        devServer: {
          proxy: {
            '/api/auth': {
              target: 'http://localhost:3001',
              changeOrigin: true,
              pathRewrite: { '^/api/auth': '/api/auth' },
              logLevel: "debug",
            },
          },
        },
      };
    },
  };
};
