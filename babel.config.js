module.exports = {
  presets: [
    [
      require.resolve('@docusaurus/core/lib/babel/preset'),
      {
        // Enable modern ES module syntax
        modules: false, // Don't transform ES modules to CommonJS
      },
    ],
  ],
};